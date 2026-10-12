// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 elem3n7ary
//
// Host-side test suite for the elNoise VST3. It loads the built .vst3 like a DAW would and checks
// parameters, state, DSP behaviour, robustness, timing and the editor's lifecycle.
//   usage: elnoise_tests <path to elNoise.vst3 binary>
#define NOMINMAX
#include <windows.h>
#include <psapi.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "pluginterfaces/base/ipluginbase.h"
#include "pluginterfaces/gui/iplugview.h"
#include "pluginterfaces/gui/iplugviewcontentscalesupport.h"
#include "pluginterfaces/vst/ivstaudioprocessor.h"
#include "pluginterfaces/vst/ivstcomponent.h"
#include "pluginterfaces/vst/ivsteditcontroller.h"
#include "public.sdk/source/common/memorystream.h"
#include "public.sdk/source/vst/hosting/parameterchanges.h"
#include "public.sdk/source/vst/hosting/hostclasses.h"
#include "pluginterfaces/vst/ivstprocesscontext.h"

using namespace Steinberg;
using namespace Steinberg::Vst;

static int g_fail = 0, g_pass = 0, g_cycles = 100;
static void check (bool ok, const char* name, const char* fmt = "", double a = 0, double b = 0, double c = 0)
{
	char detail[200];
	snprintf (detail, sizeof detail, fmt, a, b, c);
	printf ("  [%s] %s %s\n", ok ? "PASS" : "FAIL", name, detail);
	(ok ? g_pass : g_fail)++;
}

static IPluginFactory* g_factory = nullptr;
static TUID g_cid;

enum { kMethod = 98, kBypass = 99, kSuppression = 100, kVadThr = 101, kVadGrace = 102, kMono = 103, kEco = 104 };

struct Plugin
{
	IComponent* comp = nullptr;
	IAudioProcessor* proc = nullptr;
	IEditController* ctrl = nullptr;
	int channels = 2;

	bool create (double sr, int ch, int maxBlock = 4096, bool activate = true)
	{
		channels = ch;
		if (g_factory->createInstance (g_cid, IComponent::iid, reinterpret_cast<void**> (&comp)) != kResultOk || !comp)
			return false;
		comp->initialize (nullptr);
		comp->queryInterface (IAudioProcessor::iid, reinterpret_cast<void**> (&proc));
		comp->queryInterface (IEditController::iid, reinterpret_cast<void**> (&ctrl));
		SpeakerArrangement arr = ch == 1 ? SpeakerArr::kMono : SpeakerArr::kStereo;
		proc->setBusArrangements (&arr, 1, &arr, 1);
		ProcessSetup ps {kRealtime, kSample32, maxBlock, sr};
		proc->setupProcessing (ps);
		if (activate)
		{
			comp->setActive (true);
			proc->setProcessing (true);
		}
		return true;
	}
	void destroy ()
	{
		if (!comp)
			return;
		proc->setProcessing (false);
		comp->setActive (false);
		comp->terminate ();
		if (ctrl) ctrl->release ();
		if (proc) proc->release ();
		comp->release ();
		comp = nullptr; proc = nullptr; ctrl = nullptr;
	}
	void set (ParamID id, double normalized) { ctrl->setParamNormalized (id, normalized); }
	double get (ParamID id) { return ctrl->getParamNormalized (id); }
	// process n samples; in/out are per channel
	void process (float** in, float** out, int n, IParameterChanges* pc = nullptr)
	{
		AudioBusBuffers ib {}, ob {};
		ib.numChannels = ob.numChannels = channels;
		ib.channelBuffers32 = in;
		ob.channelBuffers32 = out;
		ProcessData pd {};
		pd.processMode = kRealtime;
		pd.symbolicSampleSize = kSample32;
		pd.numSamples = n;
		pd.numInputs = pd.numOutputs = 1;
		pd.inputs = &ib;
		pd.outputs = &ob;
		pd.inputParameterChanges = pc;
		proc->process (pd);
	}
};

//------------------------------------------------------------------------------------------------
// signals
static std::vector<float> makeSignal (int n, double sr, double noiseRms, bool speech, unsigned seed = 7)
{
	std::vector<float> x (n);
	std::mt19937 rng (seed);
	std::normal_distribution<float> nd (0.f, 1.f);
	double ph = 0;
	for (int i = 0; i < n; ++i)
	{
		double t = i / sr;
		float s = 0.f;
		if (speech)
		{
			double env = std::fmod (t, 2.0) < 1.0 ? 1.0 : 0.0;
			ph += 2 * 3.14159265358979 * (140.0 + 30.0 * std::sin (t * 3)) / sr;
			for (int h = 1; h <= 14; ++h)
				s += static_cast<float> (std::sin (ph * h) / h * (h < 5 ? 1.0 : 0.5));
			s *= static_cast<float> (0.12 * env);
		}
		x[i] = s + nd (rng) * static_cast<float> (noiseRms);
	}
	return x;
}

static double rmsDb (const std::vector<float>& v, size_t from, size_t to)
{
	double s = 0;
	to = std::min (to, v.size ());
	for (size_t i = from; i < to; ++i)
		s += double (v[i]) * v[i];
	return 10 * std::log10 (s / std::max<size_t> (1, to - from) + 1e-20);
}

// runs a plugin over the whole buffer with the given block size; hook is called before every block
static std::vector<std::vector<float>> run (Plugin& p, const std::vector<float>& x, int block,
                                            const std::function<void (int)>& hook = nullptr, bool stereoSame = true)
{
	const int n = static_cast<int> (x.size ());
	std::vector<std::vector<float>> out (p.channels, std::vector<float> (n, 0.f));
	std::vector<float> inBuf (p.channels * block), outBuf (p.channels * block);
	int bi = 0;
	for (int pos = 0; pos < n; pos += block, ++bi)
	{
		int m = std::min (block, n - pos);
		float* in[2];
		float* ou[2];
		for (int c = 0; c < p.channels; ++c)
		{
			in[c] = &inBuf[c * block];
			ou[c] = &outBuf[c * block];
			for (int i = 0; i < m; ++i)
				in[c][i] = x[pos + i];
		}
		if (hook)
			hook (bi);
		p.process (in, ou, m);
		for (int c = 0; c < p.channels; ++c)
			for (int i = 0; i < m; ++i)
				out[c][pos + i] = ou[c][i];
	}
	(void)stereoSame;
	return out;
}

static bool allFinite (const std::vector<float>& v)
{
	for (float f : v)
		if (!std::isfinite (f))
			return false;
	return true;
}

static const char* kMethodNames[3] = {"Speex", "RNNoise", "RNNoise Lite"};
static double mnorm (int m) { return m / 2.0; }

//------------------------------------------------------------------------------------------------
static void testFactoryAndParams ()
{
	printf ("\n[1] factory + parameters\n");
	PFactoryInfo fi;
	g_factory->getFactoryInfo (&fi);
	check (std::string (fi.vendor) == "elem3n7ary", "vendor is elem3n7ary", "");
	PClassInfo ci;
	bool found = false;
	for (int i = 0; i < g_factory->countClasses (); ++i)
	{
		g_factory->getClassInfo (i, &ci);
		if (std::string (ci.category) == "Audio Module Class")
		{
			found = true;
			memcpy (g_cid, ci.cid, sizeof (TUID));
			check (std::string (ci.name) == "elNoise", "class name is elNoise", "");
		}
	}
	check (found, "audio module class present", "");

	Plugin p;
	check (p.create (48000, 2), "plugin instance created", "");
	check (p.ctrl->getParameterCount () == 7, "7 parameters", "(got %.0f)", p.ctrl->getParameterCount ());
	struct Exp { ParamID id; double def; const char* what; };
	const Exp exp[] = {{kMethod, 0.0, "method default = Speex"}, {kBypass, 0.0, "bypass default = off"},
	                   {kSuppression, 0.5, "suppression default = -30 dB"}, {kVadThr, 0.0, "VAD threshold default = 0 (off)"},
	                   {kVadGrace, 0.4, "grace default = 200 ms"}};
	for (auto& e : exp)
		check (std::fabs (p.get (e.id) - e.def) < 1e-9, e.what, "(got %.4f)", p.get (e.id));
	check (std::fabs (p.ctrl->normalizedParamToPlain (kSuppression, 0.5) + 30.0) < 1e-6, "suppression plain value -30 dB", "");
	check (std::fabs (p.ctrl->normalizedParamToPlain (kVadGrace, 0.4) - 200.0) < 1e-6, "grace plain value 200 ms", "");
	bool bypassFlag = false, autoAll = true;
	int methodSteps = -1;
	for (int i = 0; i < p.ctrl->getParameterCount (); ++i)
	{
		ParameterInfo pi {};
		p.ctrl->getParameterInfo (i, pi);
		if (pi.id == kBypass)
			bypassFlag = (pi.flags & ParameterInfo::kIsBypass) != 0;
		if (pi.id == kMethod)
			methodSteps = pi.stepCount;
		autoAll = autoAll && (pi.flags & ParameterInfo::kCanAutomate);
	}
	check (!bypassFlag, "our Bypass is not the host bypass (host disable must stay authoritative)", "");
	check (methodSteps == 2, "method has 3 entries (stepCount 2)", "(got %.0f)", methodSteps);
	check (autoAll, "all parameters are automatable", "");
	check (p.proc->getLatencySamples () == 480, "reported latency = 480 samples (10 ms at 48 kHz)", "(got %.0f)", p.proc->getLatencySamples ());
	check (p.proc->canProcessSampleSize (kSample32) == kResultTrue && p.proc->canProcessSampleSize (kSample64) != kResultTrue,
	       "32-bit float only", "");
	p.destroy ();
}

static void testState ()
{
	printf("\n[2] state save / load\n");
	Plugin a, b;
	a.create (48000, 2);
	a.set (kMethod, mnorm (2)); a.set (kBypass, 1.0); a.set (kSuppression, 0.25); a.set (kVadThr, 0.6); a.set (kVadGrace, 0.3); a.set (kMono, 1.0); a.set (kEco, 1.0);
	MemoryStream ms;
	check (a.comp->getState (&ms) == kResultOk, "getState ok", "");
	b.create (48000, 2);
	int64 pos;
	ms.seek (0, IBStream::kIBSeekSet, &pos);
	check (b.comp->setState (&ms) == kResultOk, "setState ok", "");
	const ParamID ids[] = {kMethod, kBypass, kSuppression, kVadThr, kVadGrace, kMono, kEco};
	bool same = true;
	for (ParamID id : ids)
		same = same && std::fabs (a.get (id) - b.get (id)) < 1e-9;
	check (same, "all 7 parameters restored exactly", "");
	// an old-style state with only the suppression value must still load
	MemoryStream old;
	double d = -12.0;
	int64 w;
	old.write (&d, sizeof d, nullptr);
	old.seek (0, IBStream::kIBSeekSet, &w);
	Plugin c;
	c.create (48000, 2);
	check (c.comp->setState (&old) == kResultOk && std::fabs (c.ctrl->normalizedParamToPlain (kSuppression, c.get (kSuppression)) + 12.0) < 1e-6,
	       "legacy 1.0 state (suppression only) loads", "");
	a.destroy (); b.destroy (); c.destroy ();
}

static void testBlockSizeInvariance ()
{
	printf ("\n[3] output must not depend on the host's block size\n");
	const auto x = makeSignal (48000 * 4, 48000, 0.004, true);
	for (int m = 0; m < 3; ++m)
	{
		Plugin ref;
		ref.create (48000, 2);
		ref.set (kMethod, mnorm (m));
		auto r = run (ref, x, 480);
		ref.destroy ();
		for (int bs : {1, 37, 100, 1000, 4096})
		{
			Plugin p;
			p.create (48000, 2);
			p.set (kMethod, mnorm (m));
			auto o = run (p, x, bs);
			p.destroy ();
			bool eq = o[0] == r[0] && o[1] == r[1];
			char nm[100];
			snprintf (nm, sizeof nm, "%s: block %d identical to block 480", kMethodNames[m], bs);
			check (eq, nm, "");
		}
	}
}

static void testNoiseReduction ()
{
	printf ("\n[4] noise reduction and speech preservation (synthetic voice, 48 kHz)\n");
	const int n = 48000 * 8;
	const auto noisy = makeSignal (n, 48000, 0.003, true);
	const auto clean = makeSignal (n, 48000, 0.0, true);
	// pause = second half of every 2 s period, speech = first half; skip the first 0.5 s
	auto pauseDb = [&] (const std::vector<float>& v) { return rmsDb (v, 48000 * 3 + 12000, 48000 * 3 + 40000); };
	auto speechDb = [&] (const std::vector<float>& v) { return rmsDb (v, 48000 * 2 + 4800, 48000 * 2 + 40000); };
	const double inPause = pauseDb (noisy), inSpeech = speechDb (noisy);
	printf ("  input: pause %.1f dBFS, speech %.1f dBFS\n", inPause, inSpeech);
	for (int m = 0; m < 3; ++m)
	{
		Plugin p;
		p.create (48000, 2);
		p.set (kMethod, mnorm (m));
		p.set (kSuppression, 0.5); // -30 dB
		auto o = run (p, noisy, 480);
		p.destroy ();
		char nm[120];
		snprintf (nm, sizeof nm, "%s: pauses at least 15 dB quieter", kMethodNames[m]);
		check (inPause - pauseDb (o[0]) > 15.0, nm, "(%.1f dB reduction)", inPause - pauseDb (o[0]));
		snprintf (nm, sizeof nm, "%s: speech level within 3 dB", kMethodNames[m]);
		check (std::fabs (speechDb (o[0]) - inSpeech) < 3.0, nm, "(%.1f dB change)", speechDb (o[0]) - inSpeech);
		snprintf (nm, sizeof nm, "%s: identical input gives the same left/right (only the -88 dB dither differs)", kMethodNames[m]);
		float md = 0;
		for (size_t i = 0; i < o[0].size (); ++i) md = std::max (md, std::fabs (o[0][i] - o[1][i]));
		check (md < 5e-4f, nm, "(max difference %.6f)", md);
	}
	// mono must equal the left channel of the stereo run
	for (int m = 0; m < 3; ++m)
	{
		Plugin s, mo;
		s.create (48000, 2); s.set (kMethod, mnorm (m));
		mo.create (48000, 1); mo.set (kMethod, mnorm (m));
		auto os = run (s, noisy, 480), om = run (mo, noisy, 480);
		s.destroy (); mo.destroy ();
		char nm[100];
		snprintf (nm, sizeof nm, "%s: mono output equals stereo left", kMethodNames[m]);
		check (os[0] == om[0], nm, "");
	}
	(void)clean;
}

static void testGate ()
{
	printf ("\n[5] voice gate (RNNoise)\n");
	const int n = 48000 * 8;
	const auto noisy = makeSignal (n, 48000, 0.003, true);
	auto pauseDb = [&] (const std::vector<float>& v) { return rmsDb (v, 48000 * 3 + 12000, 48000 * 3 + 40000); };
	auto speechDb = [&] (const std::vector<float>& v) { return rmsDb (v, 48000 * 2 + 4800, 48000 * 2 + 40000); };
	for (int m = 1; m <= 2; ++m)
	{
		Plugin off, on;
		off.create (48000, 2); off.set (kMethod, mnorm (m));
		on.create (48000, 2); on.set (kMethod, mnorm (m)); on.set (kVadThr, 0.6); on.set (kVadGrace, 0.0);
		auto a = run (off, noisy, 480), b = run (on, noisy, 480);
		off.destroy (); on.destroy ();
		char nm[100];
		snprintf (nm, sizeof nm, "%s: gate 60%% makes pauses no louder", kMethodNames[m]);
		check (pauseDb (b[0]) <= pauseDb (a[0]) + 0.5, nm, "(%.1f -> %.1f dBFS)", pauseDb (a[0]), pauseDb (b[0]));
		snprintf (nm, sizeof nm, "%s: gate 60%% closes to the noise floor (< -84 dBFS)", kMethodNames[m]);
		check (pauseDb (b[0]) < -84.0, nm, "(%.1f dBFS)", pauseDb (b[0]));
		snprintf (nm, sizeof nm, "%s: gate 60%% leaves speech intact (within 3 dB)", kMethodNames[m]);
		check (std::fabs (speechDb (b[0]) - speechDb (a[0])) < 3.0, nm, "(%.1f dB)", speechDb (b[0]) - speechDb (a[0]));
		snprintf (nm, sizeof nm, "%s: output finite", kMethodNames[m]);
		check (allFinite (a[0]) && allFinite (b[0]), nm, "");
	}
}

static void testBypassAndLatency ()
{
	printf ("\n[6] bypass and latency\n");
	for (int m = 0; m < 3; ++m)
	{
		Plugin p;
		p.create (48000, 2);
		p.set (kMethod, mnorm (m));
		p.set (kBypass, 1.0);
		std::vector<float> x (48000, 0.f);
		x[10000] = 0.5f;
		auto o = run (p, x, 480);
		p.destroy ();
		int peak = static_cast<int> (std::max_element (o[0].begin (), o[0].end (), [] (float a, float b) { return std::fabs (a) < std::fabs (b); }) - o[0].begin ());
		char nm[100];
		snprintf (nm, sizeof nm, "%s: bypassed impulse arrives exactly 480 samples later", kMethodNames[m]);
		check (peak == 10480 && std::fabs (o[0][peak] - 0.5f) < 1e-6f, nm, "(peak at %.0f)", peak);
	}
	// bypass on/off during processing: no clicks (largest sample step stays small for a sine)
	Plugin p;
	p.create (48000, 2);
	std::vector<float> x (48000 * 2);
	for (size_t i = 0; i < x.size (); ++i)
		x[i] = 0.1f * std::sin (2 * 3.14159265f * 440.f * i / 48000.f);
	auto o = run (p, x, 480, [&] (int bi) { if (bi % 20 == 10) p.set (kBypass, (bi / 20) % 2 ? 0.0 : 1.0); });
	p.destroy ();
	float maxStep = 0;
	for (size_t i = 12000; i + 1 < o[0].size (); ++i)
		maxStep = std::max (maxStep, std::fabs (o[0][i + 1] - o[0][i]));
	check (maxStep < 0.1f, "toggling bypass does not click", "(largest sample step %.3f)", maxStep);

	// every engine: leaving and entering bypass is click-free, and a bypassed engine costs (almost) nothing
	for (int m = 0; m < 3; ++m)
	{
		Plugin q;
		q.create (48000, 2);
		q.set (kMethod, mnorm (m));
		const auto sp = makeSignal (48000 * 6, 48000, 0.004, true);
		auto oq = run (q, sp, 480, [&] (int bi) { q.set (kBypass, (bi / 50) % 2 ? 1.0 : 0.0); });
		float step = 0;
		for (size_t i = 12000; i + 1 < oq[0].size (); ++i)
			step = std::max (step, std::fabs (oq[0][i + 1] - oq[0][i]));
		char nm[120];
		snprintf (nm, sizeof nm, "%s: switching bypass on/off on live speech stays smooth", kMethodNames[m]);
		check (allFinite (oq[0]) && step < 0.25f, nm, "(largest sample step %.3f)", step);

		q.set (kBypass, 1.0);
		std::vector<float> a (480, 0.05f), b (480, 0.05f), oa (480), ob (480);
		float* in[2] = {a.data (), b.data ()};
		float* out[2] = {oa.data (), ob.data ()};
		for (int i = 0; i < 400; ++i) q.process (in, out, 480); // settle fully into bypass
		LARGE_INTEGER fq, t0, t1;
		QueryPerformanceFrequency (&fq);
		QueryPerformanceCounter (&t0);
		for (int i = 0; i < 2000; ++i) q.process (in, out, 480);
		QueryPerformanceCounter (&t1);
		const double msPer = 1000.0 * (t1.QuadPart - t0.QuadPart) / fq.QuadPart / 2000.0;
		snprintf (nm, sizeof nm, "%s: bypassed block takes only %.4f ms", kMethodNames[m], msPer);
		check (msPer < 0.02, nm, "");
		q.destroy ();
	}
}

static void testRobustness ()
{
	printf ("\n[7] robustness\n");
	const int n = 48000 * 6;
	const auto x = makeSignal (n, 48000, 0.004, true);
	// random method / parameter changes while processing
	for (int m = 0; m < 1; ++m)
	{
		Plugin p;
		p.create (48000, 2);
		std::mt19937 rng (3);
		auto o = run (p, x, 480, [&] (int bi) {
			if (bi % 7 == 0) p.set (kMethod, mnorm (rng () % 3));
			if (bi % 11 == 0) p.set (kSuppression, (rng () % 1000) / 999.0);
			if (bi % 13 == 0) p.set (kVadThr, (rng () % 101) / 100.0);
			if (bi % 17 == 0) p.set (kVadGrace, (rng () % 51) / 50.0);
			if (bi % 19 == 0) p.set (kBypass, rng () % 2);
		});
		p.destroy ();
		float peak = 0;
		for (float f : o[0]) peak = std::max (peak, std::fabs (f));
		check (allFinite (o[0]) && allFinite (o[1]), "random parameter/method hammering: output finite", "");
		check (peak < 1.5f, "random hammering: output bounded", "(peak %.2f)", peak);
	}
	// NaN / Inf / huge input
	for (int m = 0; m < 3; ++m)
	{
		Plugin p;
		p.create (48000, 2);
		p.set (kMethod, mnorm (m));
		auto bad = x;
		for (int i = 20000; i < 20050; ++i) bad[i] = std::numeric_limits<float>::quiet_NaN ();
		for (int i = 30000; i < 30050; ++i) bad[i] = std::numeric_limits<float>::infinity ();
		for (int i = 40000; i < 40050; ++i) bad[i] = 1e9f;
		auto o = run (p, bad, 480);
		p.destroy ();
		char nm[100];
		snprintf (nm, sizeof nm, "%s: NaN / Inf / 1e9 input never produces non-finite output", kMethodNames[m]);
		check (allFinite (o[0]), nm, "");
	}
	// digital silence in -> never digital silence out (dither floor), but very quiet
	for (int m = 0; m < 3; ++m)
	{
		Plugin p;
		p.create (48000, 2);
		p.set (kMethod, mnorm (m));
		std::vector<float> z (48000 * 2, 0.f);
		auto o = run (p, z, 480);
		p.destroy ();
		size_t zeros = 0;
		for (size_t i = 24000; i < o[0].size (); ++i) zeros += o[0][i] == 0.f;
		double db = rmsDb (o[0], 24000, o[0].size ());
		char nm[100];
		snprintf (nm, sizeof nm, "%s: silence in -> faint noise floor out (no exact zeros)", kMethodNames[m]);
		check (zeros < 100 && db > -100.0 && db < -80.0, nm, "(%.0f zeros, %.1f dBFS)", double (zeros), db);
	}
	// sample rates
	for (double sr : {8000.0, 16000.0, 22050.0, 44100.0, 88200.0, 96000.0, 192000.0})
	{
		const auto y = makeSignal (static_cast<int> (sr * 3.2), sr, 0.003, true);
		for (int m = 0; m < 3; ++m)
		{
			Plugin p;
			p.create (sr, 2, 8192);
			p.set (kMethod, mnorm (m));
			auto o = run (p, y, 480);
			p.destroy ();
			char nm[120];
			snprintf (nm, sizeof nm, "%.0f Hz, %s: finite and speech passes", sr, kMethodNames[m]);
			double inS = rmsDb (y, static_cast<size_t> (sr * 2.1), static_cast<size_t> (sr * 2.9)), outS = rmsDb (o[0], static_cast<size_t> (sr * 2.1), static_cast<size_t> (sr * 2.9));
			check (allFinite (o[0]) && std::fabs (outS - inS) < 6.0, nm, "(%.1f dB)", outS - inS);
		}
	}
}

static void testAutomation ()
{
	printf ("\n[8] host automation through ProcessData\n");
	Plugin p;
	p.create (48000, 2);
	std::vector<float> buf (480, 0.f), obuf (480, 0.f);
	float* in[2] = {buf.data (), buf.data ()};
	float* out[2] = {obuf.data (), obuf.data ()};
	ParameterChanges pc (6);
	int32 idx;
	int32 pi;
	pc.addParameterData (kMethod, idx)->addPoint (0, 1.0, pi);
	pc.addParameterData (kBypass, idx)->addPoint (0, 1.0, pi);
	pc.addParameterData (kVadThr, idx)->addPoint (0, 0.5, pi);
	pc.addParameterData (kVadGrace, idx)->addPoint (0, 0.1, pi);
	pc.addParameterData (kSuppression, idx)->addPoint (0, 0.2, pi);
	p.process (in, out, 480, &pc);
	check (std::fabs (p.get (kMethod) - 1.0) < 1e-9, "automated method change arrives", "");
	check (std::fabs (p.get (kBypass) - 1.0) < 1e-9, "automated bypass arrives", "");
	check (std::fabs (p.get (kVadThr) - 0.5) < 1e-9, "automated VAD threshold arrives", "");
	check (std::fabs (p.get (kVadGrace) - 0.1) < 1e-9, "automated grace arrives", "");
	check (std::fabs (p.get (kSuppression) - 0.2) < 1e-9, "automated suppression arrives", "");
	p.destroy ();
}

//------------------------------------------------------------------------------------------------
static void testTiming ()
{
	printf ("\n[9] real-time cost (stereo, 480-sample blocks = 10 ms budget)\n");
	const auto x = makeSignal (48000 * 10, 48000, 0.004, true);
	for (int m = 0; m < 3; ++m)
	{
		Plugin p;
		p.create (48000, 2);
		p.set (kMethod, mnorm (m));
		std::vector<float> a (480), b (480), oa (480), ob (480);
		float* in[2] = {a.data (), b.data ()};
		float* out[2] = {oa.data (), ob.data ()};
		std::vector<double> ms;
		LARGE_INTEGER fq, t0, t1;
		QueryPerformanceFrequency (&fq);
		for (int blk = 0; blk < 1000; ++blk)
		{
			for (int i = 0; i < 480; ++i) a[i] = b[i] = x[blk * 480 + i];
			QueryPerformanceCounter (&t0);
			p.process (in, out, 480);
			QueryPerformanceCounter (&t1);
			if (blk > 20) ms.push_back (1000.0 * (t1.QuadPart - t0.QuadPart) / fq.QuadPart);
		}
		p.destroy ();
		std::sort (ms.begin (), ms.end ());
		double avg = 0;
		for (double v : ms) avg += v;
		avg /= ms.size ();
		double p999 = ms[size_t (ms.size () * 0.999)], mx = ms.back ();
		char nm[100];
		snprintf (nm, sizeof nm, "%s: average %.3f ms (%.1f%% of a core), p99.9 %.3f ms, max %.3f ms", kMethodNames[m], avg, avg / 10.0 * 100.0, p999, mx);
		check (avg < 2.0 && p999 < 6.0, nm, "");
	}
}

static size_t privateBytes ()
{
	PROCESS_MEMORY_COUNTERS_EX pm {};
	GetProcessMemoryInfo (GetCurrentProcess (), reinterpret_cast<PROCESS_MEMORY_COUNTERS*> (&pm), sizeof pm);
	return pm.PrivateUsage;
}

static void testSoak ()
{
	printf ("\n[10] soak: 10 minutes of audio per engine, memory must stay flat\n");
	const auto x = makeSignal (48000 * 10, 48000, 0.004, true);
	for (int m = 0; m < 3; ++m)
	{
		Plugin p;
		p.create (48000, 2);
		p.set (kMethod, mnorm (m));
		std::vector<float> a (480), oa (480), ob (480);
		float* in[2] = {a.data (), a.data ()};
		float* out[2] = {oa.data (), ob.data ()};
		size_t base = 0;
		bool finite = true;
		for (int sec = 0; sec < 600; ++sec)
		{
			for (int blk = 0; blk < 100; ++blk)
			{
				for (int i = 0; i < 480; ++i) a[i] = x[((sec % 10) * 100 + blk) * 480 + i];
				p.process (in, out, 480);
				finite = finite && std::isfinite (oa[0]) && std::isfinite (ob[479]);
			}
			if (sec == 20) base = privateBytes ();
		}
		long long growS = static_cast<long long> (privateBytes ()) - static_cast<long long> (base);
		size_t grow = growS > 0 ? static_cast<size_t> (growS) : 0;
		p.destroy ();
		char nm[100];
		snprintf (nm, sizeof nm, "%s: 600 s processed, finite, memory growth %.2f MB", kMethodNames[m], grow / 1048576.0);
		check (finite && grow < 2 * 1048576, nm, "");
	}
}

static LRESULT CALLBACK hostProc (HWND h, UINT m, WPARAM w, LPARAM l) { return DefWindowProcW (h, m, w, l); }

static void testEditorLifecycle ()
{
	printf ("\n[11] editor: open / close 100 times while audio and parameters run on other threads\n");
	Plugin p;
	p.create (48000, 2);
	WNDCLASSW wc {};
	wc.lpfnWndProc = hostProc;
	wc.hInstance = GetModuleHandleW (nullptr);
	wc.lpszClassName = L"ElnoiseTestHost";
	RegisterClassW (&wc);

	std::atomic<bool> stop {false};
	std::atomic<long long> blocks {0};
	std::thread audio ([&] {
		std::vector<float> a (480, 0.01f), oa (480), ob (480);
		float* in[2] = {a.data (), a.data ()};
		float* out[2] = {oa.data (), ob.data ()};
		std::mt19937 rng (1);
		while (!stop)
		{
			for (int i = 0; i < 480; ++i) a[i] = 0.05f * std::sin (blocks * 480 + i) + 0.001f * (float (rng () % 2000) / 1000.f - 1.f);
			p.process (in, out, 480);
			++blocks;
			if (blocks % 4 == 0) std::this_thread::sleep_for (std::chrono::milliseconds (1));
		}
	});
	std::thread params ([&] {
		std::mt19937 rng (9);
		while (!stop)
		{
			p.set (kMethod, mnorm (rng () % 3));
			p.set (kVadThr, (rng () % 101) / 100.0);
			p.set (kSuppression, (rng () % 1000) / 999.0);
			std::this_thread::sleep_for (std::chrono::milliseconds (5));
		}
	});

	auto cycle = [&] {
		RECT r {0, 0, 560, 424};
		AdjustWindowRect (&r, WS_OVERLAPPEDWINDOW, FALSE);
		HWND h = CreateWindowW (L"ElnoiseTestHost", L"t", WS_OVERLAPPEDWINDOW | WS_VISIBLE, 50, 50, r.right - r.left, r.bottom - r.top, nullptr, nullptr, wc.hInstance, nullptr);
		IPlugView* v = p.ctrl->createView ("editor");
		bool ok = v && v->attached (h, "HWND") == kResultOk;
		DWORD until = GetTickCount () + 120;
		while (GetTickCount () < until)
		{
			MSG m;
			while (PeekMessageW (&m, nullptr, 0, 0, PM_REMOVE)) { TranslateMessage (&m); DispatchMessageW (&m); }
			Sleep (2);
		}
		if (v)
		{
			v->removed ();
			v->release ();
		}
		DestroyWindow (h);
		return ok;
	};
	for (int i = 0; i < 10; ++i) cycle (); // warm up (driver / font caches settle)
	DWORD hb = 0;
	GetProcessHandleCount (GetCurrentProcess (), &hb);
	const DWORD gdi0 = GetGuiResources (GetCurrentProcess (), GR_GDIOBJECTS), usr0 = GetGuiResources (GetCurrentProcess (), GR_USEROBJECTS);
	// The graphics driver moves memory around in big jumps, so a start/end difference is meaningless. A real leak
	// raises the *lowest* memory level over time, so compare the lowest level of the first and the last third.
	std::vector<double> samples;
	bool allOk = true;
	for (int i = 1; i <= g_cycles; ++i)
	{
		allOk = cycle () && allOk;
		if (i % 5 == 0)
			samples.push_back (privateBytes () / 1048576.0);
	}
	DWORD he = 0;
	GetProcessHandleCount (GetCurrentProcess (), &he);
	const DWORD gdi1 = GetGuiResources (GetCurrentProcess (), GR_GDIOBJECTS), usr1 = GetGuiResources (GetCurrentProcess (), GR_USEROBJECTS);
	stop = true;
	audio.join ();
	params.join ();
	p.destroy ();
	const size_t third = std::max<size_t> (1, samples.size () / 3);
	const double lowFirst = *std::min_element (samples.begin (), samples.begin () + third);
	const double lowLast = *std::min_element (samples.end () - third, samples.end ());
	const double first = samples.front (), last = samples.back ();
	const double leakPerCycle = (lowLast - lowFirst) / std::max (1, g_cycles - int (third) * 5);
	check (allOk, "editor attached every time without failure", "(%.0f cycles)", double (g_cycles));
	check (blocks > 200, "audio kept running while the editor opened/closed", "(%.0f blocks)", double (blocks));
	check (he <= hb + 10, "no handle leak", "(%.0f -> %.0f)", double (hb), double (he));
	check (gdi1 <= gdi0 + 5 && usr1 <= usr0 + 5, "no GDI / USER object leak", "(GDI %.0f -> %.0f)", double (gdi0), double (gdi1));
	printf ("  memory samples: first %.1f MB, last %.1f MB; lowest level first third %.1f MB, last third %.1f MB => %.3f MB per cycle\n", first, last, lowFirst, lowLast, leakPerCycle);
	// The graphics driver makes private memory jump by tens of MB between runs, so only a gross leak can be judged here;
	// handles and GDI/USER objects (checked above) are the reliable leak detectors.
	check (leakPerCycle < 1.0, "no gross memory growth per editor open (< 1 MB per cycle)", "");
}

static void testScaling ()
{
	printf ("\n[12] editor size and DPI scaling\n");
	Plugin p;
	p.create (48000, 2);
	IPlugView* v = p.ctrl->createView ("editor");
	check (v != nullptr, "editor view is created", "");
	ViewRect r {};
	v->getSize (&r);
	check (r.getWidth () == 560 && r.getHeight () == 424, "default size is 560 x 424", "(%.0f x %.0f)", r.getWidth (), r.getHeight ());
	IPlugViewContentScaleSupport* cs = nullptr;
	check (v->queryInterface (IPlugViewContentScaleSupport::iid, reinterpret_cast<void**> (&cs)) == kResultOk && cs,
	       "editor supports IPlugViewContentScaleSupport", "");
	if (cs)
	{
		struct Case { float f; int w, h; } cases[] = {{1.25f, 700, 530}, {1.5f, 840, 636}, {2.0f, 1120, 848}, {10.f, 2240, 1696}, {0.5f, 560, 424}};
		for (auto& c : cases)
		{
			cs->setContentScaleFactor (c.f);
			ViewRect q {};
			v->getSize (&q);
			ViewRect t {0, 0, 1, 1};
			v->checkSizeConstraint (&t);
			char nm[100];
			snprintf (nm, sizeof nm, "scale %.2f gives %d x %d (extreme values are clamped to 1x..4x)", c.f, c.w, c.h);
			check (q.getWidth () == c.w && q.getHeight () == c.h && t.getWidth () == c.w && t.getHeight () == c.h, nm, "(%.0f x %.0f)", q.getWidth (), q.getHeight ());
		}
		cs->release ();
	}
	v->release ();
	p.destroy ();
}


// stereo run with different signals on L and R
static std::vector<std::vector<float>> runLR (Plugin& p, const std::vector<float>& l, const std::vector<float>& r, int block,
                                              const std::function<void (int)>& hook = nullptr)
{
	const int n = static_cast<int> (l.size ());
	std::vector<std::vector<float>> out (2, std::vector<float> (n, 0.f));
	std::vector<float> ib (2 * block), ob (2 * block);
	int bi = 0;
	for (int pos = 0; pos < n; pos += block, ++bi)
	{
		const int m = std::min (block, n - pos);
		float* in[2] = {&ib[0], &ib[block]};
		float* ou[2] = {&ob[0], &ob[block]};
		for (int i = 0; i < m; ++i)
		{
			in[0][i] = l[pos + i];
			in[1][i] = r[pos + i];
		}
		if (hook)
			hook (bi);
		p.process (in, ou, m);
		for (int c = 0; c < 2; ++c)
			for (int i = 0; i < m; ++i)
				out[c][pos + i] = ou[c][i];
	}
	return out;
}

static void testMonoEco ()
{
	printf ("\n[14] mono mode and eco mode\n");
	const int n = 48000 * 4;
	auto l = makeSignal (n, 48000, 0.01, true, 3);
	auto r = makeSignal (n, 48000, 0.01, true, 99);
	for (int m = 0; m < 3; ++m)
	{
		char nm[120];
		Plugin p;
		p.create (48000, 2);
		p.set (kMethod, mnorm (m));
		p.set (kMono, 1.0);
		auto o = runLR (p, l, r, 480);
		double maxDiff = 0;
		for (int i = 24000; i < n; ++i)
			maxDiff = std::max (maxDiff, double (std::fabs (o[0][i] - o[1][i])));
		snprintf (nm, sizeof nm, "%s: mono output is identical on L and R", kMethodNames[m]);
		check (maxDiff == 0.0, nm, "(max diff %.3g)", maxDiff);
		snprintf (nm, sizeof nm, "%s: mono output is finite and not silent", kMethodNames[m]);
		check (allFinite (o[0]) && rmsDb (o[0], 24000, n) > -80, nm, "(%.1f dB)", rmsDb (o[0], 24000, n));
		p.destroy ();

		// mono must equal the normal engine fed with the mix (identical input on both channels)
		std::vector<float> mix (n);
		for (int i = 0; i < n; ++i)
			mix[i] = 0.5f * (l[i] + r[i]);
		Plugin a, b;
		a.create (48000, 2);
		b.create (48000, 2);
		a.set (kMethod, mnorm (m));
		b.set (kMethod, mnorm (m));
		a.set (kMono, 1.0);
		auto oa = runLR (a, l, r, 480);
		auto ob = runLR (b, mix, mix, 480);
		double e = 0;
		for (int i = 24000; i < n; ++i)
			e = std::max (e, double (std::fabs (oa[0][i] - ob[0][i])));
		snprintf (nm, sizeof nm, "%s: mono == engine fed with (L+R)/2", kMethodNames[m]);
		check (e < 1e-6, nm, "(max diff %.3g)", e);
		a.destroy ();
		b.destroy ();

		// switching mono on and off, and bypass on and off, in the middle of audio: nothing explodes or clicks
		Plugin t;
		t.create (48000, 2);
		t.set (kMethod, mnorm (m));
		auto ot = runLR (t, l, r, 480, [&] (int bi) {
			if (bi % 37 == 0) t.set (kMono, (bi / 37) & 1 ? 1.0 : 0.0);
			if (bi % 53 == 0) t.set (kBypass, (bi / 53) & 1 ? 1.0 : 0.0);
		});
		double peak = 0;
		for (int c = 0; c < 2; ++c)
			for (float v : ot[c])
				peak = std::max (peak, double (std::fabs (v)));
		snprintf (nm, sizeof nm, "%s: toggling mono/bypass during audio stays finite and in range", kMethodNames[m]);
		check (allFinite (ot[0]) && allFinite (ot[1]) && peak < 1.2, nm, "(peak %.3f)", peak);
		t.destroy ();
	}
	// mono costs about half
	{
		Plugin st, mo;
		st.create (48000, 2);
		mo.create (48000, 2);
		st.set (kMethod, mnorm (1));
		st.set (kMono, 0.0);
		mo.set (kMethod, mnorm (1));
		mo.set (kMono, 1.0);
		auto time = [&] (Plugin& p) {
			auto t0 = std::chrono::steady_clock::now ();
			runLR (p, l, r, 480);
			return std::chrono::duration<double> (std::chrono::steady_clock::now () - t0).count ();
		};
		time (st); time (mo); // warm up
		const double ts = time (st), tm = time (mo);
		check (tm < ts * 0.75, "RNNoise: mono needs clearly less CPU than stereo", "(stereo %.0f ms, mono %.0f ms)", ts * 1000, tm * 1000);
		st.destroy ();
		mo.destroy ();
	}
}

static void testInstancesAndLifecycle ()
{
	printf ("\n[13] many instances, activate/deactivate cycles\n");
	std::vector<Plugin> ps (8);
	bool ok = true;
	for (auto& p : ps) ok = p.create (48000, 2) && ok;
	const auto x = makeSignal (48000, 48000, 0.004, true);
	for (size_t i = 0; i < ps.size (); ++i)
	{
		ps[i].set (kMethod, mnorm (i % 3));
		auto o = run (ps[i], x, 480);
		ok = ok && allFinite (o[0]);
	}
	for (auto& p : ps) p.destroy ();
	check (ok, "8 simultaneous instances (mixed engines) all process cleanly", "");
	Plugin p;
	p.create (48000, 2);
	bool cyc = true;
	for (int i = 0; i < 25; ++i)
	{
		p.proc->setProcessing (false);
		p.comp->setActive (false);
		ProcessSetup ps2 {kRealtime, kSample32, 480, i % 2 ? 44100.0 : 48000.0};
		p.proc->setupProcessing (ps2);
		p.comp->setActive (true);
		p.proc->setProcessing (true);
		std::vector<float> a (480, 0.05f), oa (480), ob (480);
		float* in[2] = {a.data (), a.data ()};
		float* out[2] = {oa.data (), ob.data ()};
		p.process (in, out, 480);
		cyc = cyc && std::isfinite (oa[100]);
	}
	p.destroy ();
	check (cyc, "25x setActive / setupProcessing (alternating 44.1/48 kHz) cycles", "");
}

// inspect <vst3>: lists what any plugin tells a host through the public VST3 interfaces (no audio is processed)
static int inspectPlugin (const wchar_t* path)
{
	HMODULE mod = LoadLibraryW (path);
	if (!mod)
	{
		printf ("cannot load\n");
		return 2;
	}
	if (auto initDll = reinterpret_cast<bool (*) ()> (GetProcAddress (mod, "InitDll")))
		initDll ();
	auto getFactory = reinterpret_cast<IPluginFactory* (*) ()> (GetProcAddress (mod, "GetPluginFactory"));
	IPluginFactory* f = getFactory ();
	PFactoryInfo fi;
	f->getFactoryInfo (&fi);
	printf ("factory: vendor='%s' url='%s'\n", fi.vendor, fi.url);
	HostApplication host;
	auto s16 = [] (const TChar* t) {
		std::string o;
		for (; *t; ++t) o += static_cast<char> (*t < 128 ? *t : '?');
		return o;
	};
	for (int i = 0; i < f->countClasses (); ++i)
	{
		PClassInfo ci;
		f->getClassInfo (i, &ci);
		printf ("class %d: '%s' category='%s'\n", i, ci.name, ci.category);
		if (std::string (ci.category) != "Audio Module Class")
			continue;
		IComponent* comp = nullptr;
		if (f->createInstance (ci.cid, IComponent::iid, reinterpret_cast<void**> (&comp)) != kResultOk || !comp)
			continue;
		comp->initialize (&host);
		IAudioProcessor* proc = nullptr;
		comp->queryInterface (IAudioProcessor::iid, reinterpret_cast<void**> (&proc));
		IEditController* ctrl = nullptr;
		if (comp->queryInterface (IEditController::iid, reinterpret_cast<void**> (&ctrl)) != kResultOk || !ctrl)
		{
			TUID cc;
			if (comp->getControllerClassId (cc) == kResultOk)
			{
				f->createInstance (cc, IEditController::iid, reinterpret_cast<void**> (&ctrl));
				if (ctrl) ctrl->initialize (&host);
				printf ("  separate controller component\n");
			}
		}
		else
			printf ("  single-component effect (controller in the same object)\n");
		for (int dir = 0; dir < 2; ++dir)
			for (int b = 0; b < comp->getBusCount (kAudio, dir); ++b)
			{
				BusInfo bi {};
				comp->getBusInfo (kAudio, dir, b, bi);
				printf ("  %s bus %d: '%s' channels=%d default-active=%d\n", dir ? "output" : "input", b, s16 (bi.name).c_str (), bi.channelCount, (bi.flags & BusInfo::kDefaultActive) ? 1 : 0);
			}
		if (proc)
			printf ("  latency=%u samples, tail=%u samples, 32bit=%d 64bit=%d\n", proc->getLatencySamples (), proc->getTailSamples (),
			       proc->canProcessSampleSize (kSample32) == kResultTrue, proc->canProcessSampleSize (kSample64) == kResultTrue);
		if (ctrl)
		{
			printf ("  parameters: %d\n", ctrl->getParameterCount ());
			for (int p = 0; p < ctrl->getParameterCount (); ++p)
			{
				ParameterInfo pi {};
				ctrl->getParameterInfo (p, pi);
				printf ("    id=%-6u '%s' units='%s' steps=%d default=%.3f flags=[%s%s%s%s%s]\n", pi.id, s16 (pi.title).c_str (), s16 (pi.units).c_str (),
				       pi.stepCount, pi.defaultNormalizedValue, (pi.flags & ParameterInfo::kCanAutomate) ? "automate " : "",
				       (pi.flags & ParameterInfo::kIsBypass) ? "BYPASS " : "", (pi.flags & ParameterInfo::kIsReadOnly) ? "readonly " : "",
				       (pi.flags & ParameterInfo::kIsHidden) ? "hidden " : "", (pi.flags & ParameterInfo::kIsList) ? "list" : "");
			}
		}
		FUnknown* unk = nullptr;
		IProcessContextRequirements* req = nullptr;
		if (proc && proc->queryInterface (IProcessContextRequirements::iid, reinterpret_cast<void**> (&req)) == kResultOk && req)
			printf ("  process-context requirements flags=0x%x\n", req->getProcessContextRequirements ());
		(void)unk;
		if (ctrl)
		{
			IPlugView* v = ctrl->createView ("editor");
			if (v)
			{
				ViewRect r {};
				v->getSize (&r);
				printf ("  editor: %d x %d, ", r.getWidth (), r.getHeight ());
				IPlugViewContentScaleSupport* cs = nullptr;
				printf ("scale-support=%d\n", v->queryInterface (IPlugViewContentScaleSupport::iid, reinterpret_cast<void**> (&cs)) == kResultOk && cs);
				v->release ();
			}
		}
	}
	return 0;
}

int wmain (int argc, wchar_t** argv)
{
	if (argc > 2 && std::wstring (argv[2]) == L"inspect")
		return inspectPlugin (argv[1]);
	if (argc < 2)
	{
		printf ("usage: elnoise_tests <elNoise.vst3 binary>\n");
		return 2;
	}
	HMODULE mod = LoadLibraryW (argv[1]);
	if (!mod)
	{
		printf ("cannot load %ls\n", argv[1]);
		return 2;
	}
	auto initDll = reinterpret_cast<bool (*) ()> (GetProcAddress (mod, "InitDll"));
	if (initDll)
		initDll ();
	auto getFactory = reinterpret_cast<IPluginFactory* (*) ()> (GetProcAddress (mod, "GetPluginFactory"));
	g_factory = getFactory ();
	printf ("elNoise test suite\n");

	if (argc > 4 && std::wstring (argv[2]) == L"dump") // dump <method> <out.f32>: raw output for A/B comparisons between builds
	{
		PClassInfo ci;
		for (int i = 0; i < g_factory->countClasses (); ++i)
		{
			g_factory->getClassInfo (i, &ci);
			if (std::string (ci.category) == "Audio Module Class") memcpy (g_cid, ci.cid, sizeof (TUID));
		}
		Plugin p;
		p.create (48000, 2);
		p.set (kMethod, mnorm (_wtoi (argv[3])));
		auto o = run (p, makeSignal (48000 * 8, 48000, 0.003, true), 480);
		p.destroy ();
		FILE* f = _wfopen (argv[4], L"wb");
		fwrite (o[0].data (), sizeof (float), o[0].size (), f);
		fclose (f);
		return 0;
	}
	if (argc > 2 && std::wstring (argv[2]) == L"gui")
	{
		g_cycles = argc > 3 ? _wtoi (argv[3]) : 100;
		{
			Plugin t;
			PClassInfo ci;
			for (int i = 0; i < g_factory->countClasses (); ++i)
			{
				g_factory->getClassInfo (i, &ci);
				if (std::string (ci.category) == "Audio Module Class") memcpy (g_cid, ci.cid, sizeof (TUID));
			}
		}
		testEditorLifecycle ();
		printf ("\n==== %d passed, %d failed ====\n", g_pass, g_fail);
		return g_fail ? 1 : 0;
	}
	testFactoryAndParams ();
	testState ();
	testBlockSizeInvariance ();
	testNoiseReduction ();
	testGate ();
	testBypassAndLatency ();
	testRobustness ();
	testAutomation ();
	testTiming ();
	testSoak ();
	testEditorLifecycle ();
	testScaling ();
	testInstancesAndLifecycle ();
	testMonoEco ();

	printf ("\n==== %d passed, %d failed ====\n", g_pass, g_fail);
	return g_fail ? 1 : 0;
}

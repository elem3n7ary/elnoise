// SPDX-License-Identifier: GPL-3.0-or-later
// elNoise - VST3 microphone noise suppression plugin
// Copyright (C) 2026 elem3n7ary
//
// This program is free software: you can redistribute it and/or modify it under the terms of the
// GNU General Public License as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version. It is distributed WITHOUT ANY WARRANTY.
// See the LICENSE file. Bundled third-party code: see THIRD_PARTY_LICENSES/.

// elNoise: Speex noise suppression engine, mirrors the OBS "Noise Suppression" filter (Speex method).
// GUI: Direct2D / DirectWrite (GPU accelerated), frames paced by the display (DwmFlush).
#include "public.sdk/source/main/pluginfactory.h"
#include "public.sdk/source/vst/vstsinglecomponenteffect.h"
#include "public.sdk/source/vst/vstparameters.h"
#include "pluginterfaces/base/ibstream.h"
#include "pluginterfaces/vst/ivstparameterchanges.h"
#include "pluginterfaces/vst/vsttypes.h"
#include "base/source/fstreamer.h"
#include "public.sdk/source/common/pluginview.h"

#define NOMINMAX
#include <windows.h>
#include <d2d1.h>
#include <dwrite_3.h>
#include <dwmapi.h>
#include <wrl/client.h>
#pragma comment(lib, "d2d1.lib")
#pragma comment(lib, "dwrite.lib")
#pragma comment(lib, "dwmapi.lib")

#include <speex/speex_preprocess.h>
extern "C" {
#include <rnnoise.h>
DenoiseState* elnoise_create_light (void); // little model, see rnnoise_wrapper.c
}

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cwchar>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace Steinberg;
using namespace Steinberg::Vst;
using Microsoft::WRL::ComPtr;

static const FUID kPluginUID (0x5A3C1E70, 0x9B2F4D18, 0xA6C47D02, 0x3E8B5F91);

enum : ParamID { kMethod = 98, kBypass = 99, kSuppression = 100, kVadThreshold = 101, kVadGrace = 102 };
static constexpr double kMinDb = -60.0;
static constexpr double kMaxDb = 0.0;
static constexpr double kDefaultDb = -30.0; // OBS default

//------------------------------------------------------------------------
// Log-frequency spectrum analyzer + RMS level.
// The audio thread feeds it only while an editor window is open (enabled == true).
class Analyzer
{
public:
	static constexpr int N = 2048, kHop = 1024, kBands = 96;
	static constexpr float kFMin = 40.f, kFMax = 18000.f;

	Analyzer ()
	{
		const float pi = 3.14159265358979f;
		for (int i = 0; i < N; ++i)
			win[i] = 0.5f - 0.5f * std::cos (2.f * pi * i / (N - 1));
		for (int i = 0; i < N / 2; ++i)
		{
			twRe[i] = std::cos (-2.f * pi * i / N);
			twIm[i] = std::sin (-2.f * pi * i / N);
		}
		for (int i = 0, j = 0; i < N; ++i)
		{
			rev[i] = j;
			int bit = N >> 1;
			for (; j & bit; bit >>= 1)
				j ^= bit;
			j ^= bit;
		}
		for (int b = 0; b < kBands; ++b)
			bands[b] = -120.f;
		ring.assign (N, 0.f);
		setRate (48000.0);
	}

	void setRate (double sr)
	{
		rate = static_cast<float> (sr);
		float fmax = std::min (kFMax, rate * 0.45f), binHz = rate / N;
		for (int b = 0; b <= kBands; ++b)
			edge[b] = kFMin * std::pow (fmax / kFMin, float (b) / kBands) / binHz; // in bins
	}

	void setEnabled (bool e) { enabled.store (e, std::memory_order_relaxed); }

	void push (float x)
	{
		if (!enabled.load (std::memory_order_relaxed))
			return;
		ring[pos] = x;
		sumSq += x * x;
		pos = (pos + 1) & (N - 1);
		if (++cnt >= kHop)
		{
			cnt = 0;
			rmsDb.store (10.f * std::log10 (sumSq / kHop + 1e-12f), std::memory_order_relaxed);
			sumSq = 0.f;
			run ();
		}
	}

	float get (int b) const { return bands[b].load (std::memory_order_relaxed); }
	float level () const { return rmsDb.load (std::memory_order_relaxed); }

private:
	void run ()
	{
		for (int i = 0; i < N; ++i)
		{
			int r = rev[i];
			re[r] = ring[(pos + i) & (N - 1)] * win[i];
			im[r] = 0.f;
		}
		for (int len = 2; len <= N; len <<= 1)
		{
			const int half = len >> 1, step = N / len;
			for (int i = 0; i < N; i += len)
				for (int k = 0, t = 0; k < half; ++k, t += step)
				{
					int a = i + k, b = a + half;
					float tr = re[b] * twRe[t] - im[b] * twIm[t], ti = re[b] * twIm[t] + im[b] * twRe[t];
					re[b] = re[a] - tr;
					im[b] = im[a] - ti;
					re[a] += tr;
					im[a] += ti;
				}
		}
		for (int k = 0; k < N / 2; ++k)
			mag[k] = std::sqrt (re[k] * re[k] + im[k] * im[k]);

		const float norm = std::sqrt (N * 0.375f); // noise of rms r gives ~r per bin => dB ~ dBFS
		for (int b = 0; b < kBands; ++b)
		{
			float lo = edge[b], hi = edge[b + 1], m;
			if (hi - lo < 1.f)
			{
				float c = 0.5f * (lo + hi);
				int i0 = static_cast<int> (c);
				float fr = c - i0;
				m = mag[i0] * (1.f - fr) + mag[std::min (i0 + 1, N / 2 - 1)] * fr;
			}
			else
			{
				float s = 0.f;
				int i0 = static_cast<int> (std::ceil (lo)), i1 = static_cast<int> (hi);
				for (int i = i0; i <= i1; ++i)
					s += mag[i] * mag[i];
				m = std::sqrt (s / std::max (1, i1 - i0 + 1));
			}
			bands[b].store (20.f * std::log10 (m / norm + 1e-7f), std::memory_order_relaxed);
		}
	}

	float win[N], re[N], im[N], mag[N / 2], twRe[N / 2], twIm[N / 2], edge[kBands + 1];
	int rev[N];
	std::vector<float> ring;
	std::atomic<float> bands[kBands];
	std::atomic<float> rmsDb {-120.f};
	std::atomic<bool> enabled {false};
	float sumSq = 0.f, rate = 48000.f;
	int pos = 0, cnt = 0;
};

//------------------------------------------------------------------------
// Editor. Tabs: Main / About.
class SpeexView : public CPluginView
{
public:
	SpeexView (EditController* c, std::atomic<double>* db, std::atomic<bool>* bp, std::atomic<int>* mt,
	           const std::atomic<double>* sr, std::atomic<double>* vt, std::atomic<double>* vg,
	           const std::atomic<float>* vp, Analyzer* in, Analyzer* out)
	: CPluginView (nullptr), ctrl (c), dbValue (db), byp (bp), meth (mt), rate (sr), vthr (vt), vgrace (vg), vprob (vp),
	  aIn (in), aOut (out)
	{
		rect = ViewRect (0, 0, kW, kH);
		for (int b = 0; b < Analyzer::kBands; ++b)
			dIn[b] = dOut[b] = sIn[b] = sOut[b] = rIn[b] = rOut[b] = -120.f;
		lvIn = lvOut = -120.f;
	}

	tresult PLUGIN_API isPlatformTypeSupported (FIDString type) SMTG_OVERRIDE
	{
		return FIDStringsEqual (type, kPlatformTypeHWND) ? kResultTrue : kResultFalse;
	}

	tresult PLUGIN_API attached (void* parent, FIDString type) SMTG_OVERRIDE
	{
		if (!FIDStringsEqual (type, kPlatformTypeHWND))
			return kResultFalse;
		if (FAILED (D2D1CreateFactory (D2D1_FACTORY_TYPE_SINGLE_THREADED, IID_PPV_ARGS (&d2d))) ||
		    FAILED (DWriteCreateFactory (DWRITE_FACTORY_TYPE_SHARED, __uuidof (IDWriteFactory),
		                                 reinterpret_cast<IUnknown**> (dw.GetAddressOf ()))))
			return kResultFalse;
		pickFont ();
		loadBrandFont ();
		registerClass ();
		hwnd = CreateWindowExW (0, kClass, L"", WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN | WS_CLIPSIBLINGS, 0, 0, kW,
		                        kH, static_cast<HWND> (parent), nullptr, GetModuleHandleW (nullptr), this);
		if (!hwnd)
			return kResultFalse;
		aIn->setEnabled (true);
		aOut->setEnabled (true);
		running = true;
		ticker = std::thread ([this] {
			while (running)
			{
				if (FAILED (DwmFlush ()))
					Sleep (16);
				if (running && !pending.exchange (true))
					PostMessageW (hwnd, kTickMsg, 0, 0);
			}
		});
		return CPluginView::attached (parent, type);
	}

	tresult PLUGIN_API removed () SMTG_OVERRIDE
	{
		running = false;
		editing = false;
		if (ticker.joinable ())
			ticker.join ();
		aIn->setEnabled (false);
		aOut->setEnabled (false);
		if (hwnd)
		{
			DestroyWindow (hwnd);
			hwnd = nullptr;
		}
		releaseTarget ();
		layouts.clear ();
		formats.clear ();
		brandColl.Reset ();
		if (dw5 && memLoader)
			dw5->UnregisterFontFileLoader (memLoader.Get ());
		memLoader.Reset ();
		dw5.Reset ();
		return CPluginView::removed ();
	}

	tresult PLUGIN_API onSize (ViewRect* r) SMTG_OVERRIDE
	{
		if (r && hwnd)
			SetWindowPos (hwnd, nullptr, 0, 0, r->getWidth (), r->getHeight (), SWP_NOZORDER | SWP_NOMOVE);
		return kResultOk;
	}

	tresult PLUGIN_API checkSizeConstraint (ViewRect* r) SMTG_OVERRIDE
	{
		if (r)
			*r = ViewRect (0, 0, kW, kH);
		return kResultTrue;
	}

private:
	using Clr = D2D1_COLOR_F;
	enum Al { kLeft, kCenter, kRight };
	enum Grad { gBody, gHead, gFill, gSlider, gKnob, gKnobHot, gPill, gLcd, kGrads };

	static constexpr const wchar_t* kClass = L"SpeexNSView";
	static constexpr UINT kTickMsg = WM_APP + 1;
	static constexpr int kW = 560, kH = 424;
	static constexpr float kDispT = 70.f, kDispB = 272.f;
	static constexpr float kPlotL = 34.f, kPlotR = kW - 34.f, kPlotT = 102.f, kPlotB = 218.f;
	static constexpr float kDbTop = 0.f, kDbBot = -100.f;
	// Speex layout: kept exactly as in v1.1 on purpose
	static constexpr float kTrackL = 44.f, kTrackR = kW - 44.f, kTrackY = 372.f;
	static constexpr float kLcdX = kW - 156.f, kLcdY = 292.f, kLcdW = 128.f, kLcdH = 46.f;
	static constexpr float kBypX = 288.f, kBypY = 301.f, kBypW = 100.f, kBypH = 26.f;
	// RNNoise layout: two identical rows (grace, voice gate), each with a slider and a value box
	static constexpr float kRnR = 398.f, kRnY1 = 321.f, kRnY2 = 383.f;
	static constexpr float kRnLcdW = 104.f, kRnLcdX = kW - 28.f - kRnLcdW, kRnLcdH = 42.f;
	static constexpr float kRnBypW = 104.f, kRnBypH = 28.f, kRnBypX = (kW - 12.f) - 12.f - kRnBypW, kRnBypY = kDispT + 12.f;
	static constexpr float kPillX = 180.f, kPillY = 12.f, kPillW = 200.f, kPillH = 34.f;
	static constexpr float kMethodLabelDy = 12.0f, kMethodValueDy = 8.6f; // tuned so both texts share one center line
	static constexpr float kMenuX = 166.f, kMenuY = 54.f, kMenuW = 228.f, kRowH = 50.f;
	static constexpr const wchar_t* kMethodNames[3] = {L"SPEEX", L"RNNOISE", L"RNNOISE LITE"}; // grace-period slider (RNNoise modes), next to the number
	static constexpr float kTabX0 = kW - 172.f, kTabMid = kW - 94.f, kTabX1 = kW - 16.f;

	static Clr C (int r, int g, int b, int a = 255) { return D2D1::ColorF (r / 255.f, g / 255.f, b / 255.f, a / 255.f); }
	const Clr cWhite = C (240, 240, 243), cText = C (205, 205, 210), cMuted = C (128, 128, 134), cDim = C (74, 74, 80),
	          cLine = C (135, 135, 142), cOut = C (250, 250, 252);

	static void registerClass ()
	{
		static bool done = false;
		if (done)
			return;
		WNDCLASSW wc {};
		wc.style = CS_DBLCLKS;
		wc.lpfnWndProc = &SpeexView::wndProc;
		wc.hInstance = GetModuleHandleW (nullptr);
		wc.lpszClassName = kClass;
		wc.hCursor = LoadCursor (nullptr, IDC_ARROW);
		RegisterClassW (&wc);
		done = true;
	}

	//-------------------------------------------------------------- value handling
	bool rnnMode () const { return meth->load () > 0; }
	ParamID mainParam () const { return rnnMode () ? kVadThreshold : kSuppression; }
	ParamID dragParam () const { return dragTarget == 1 ? kVadGrace : mainParam (); }
	double normFor (int target) const
	{
		if (target == 1)
			return vgrace->load () / 500.0;
		return rnnMode () ? vthr->load () / 100.0 : (dbValue->load () - kMinDb) / (kMaxDb - kMinDb);
	}
	double norm () const { return normFor (0); }
	double curNorm () const { return dragging && dragTarget == 0 ? dragNorm : norm (); }
	double graceNorm () const { return dragging && dragTarget == 1 ? dragNorm : normFor (1); }
	// value shown / sent for a normalized position (whole dB, whole percent, 10 ms steps)
	int quant (int target, double t) const
	{
		if (target == 1)
			return static_cast<int> (std::lround (t * 50.0)) * 10;
		return rnnMode () ? static_cast<int> (std::lround (t * 100.0))
		                  : static_cast<int> (std::lround (kMinDb + t * (kMaxDb - kMinDb)));
	}
	double mainRange () const { return rnnMode () ? 100.0 : (kMaxDb - kMinDb); }

	// geometry that depends on the engine
	float trL () const { return kTrackL; }
	float trR () const { return rnnMode () ? kRnR : kTrackR; }
	float rowY (int target) const { return rnnMode () ? (target == 1 ? kRnY1 : kRnY2) : kTrackY; }
	void lcdRect (int target, float& x, float& y, float& w, float& h) const
	{
		if (!rnnMode ())
		{
			x = kLcdX; y = kLcdY; w = kLcdW; h = kLcdH;
			return;
		}
		x = kRnLcdX; w = kRnLcdW; h = kRnLcdH; y = rowY (target) - h / 2;
	}
	void bypassRect (float& x, float& y, float& w, float& h) const
	{
		if (rnnMode ()) { x = kRnBypX; y = kRnBypY; w = kRnBypW; h = kRnBypH; }
		else { x = kBypX; y = kBypY; w = kBypW; h = kBypH; }
	}

	// While dragging, the view tracks the value locally and only talks to the host when the
	// rounded value changes (Speex only takes whole dB anyway).
	void setNorm (double t, int target = 0)
	{
		t = std::clamp (t, 0.0, 1.0);
		dirty = true;
		if (dragging)
		{
			dragNorm = t;
			int v = quant (dragTarget, t);
			if (v != lastSentDb)
			{
				lastSentDb = v;
				ctrl->setParamNormalized (dragParam (), t);
				ctrl->performEdit (dragParam (), t);
			}
			return;
		}
		const ParamID pid = target == 1 ? kVadGrace : mainParam ();
		ctrl->setParamNormalized (pid, t);
		ctrl->performEdit (pid, t);
	}

	bool onSlider (int x, int y) const
	{
		const float ry = rowY (0);
		const bool inY = rnnMode () ? (y >= ry - 22 && y <= ry + 36) : (y >= 330 && y <= 418);
		return inY && x >= trL () - 20 && x <= trR () + 20;
	}
	bool onGrace (int x, int y) const
	{
		return rnnMode () && y >= rowY (1) - 22 && y <= rowY (1) + 24 && x >= trL () - 20 && x <= trR () + 20;
	}
	int hitLcdIdx (int x, int y) const
	{
		if (tab != 0)
			return -1;
		for (int t = 0; t < (rnnMode () ? 2 : 1); ++t)
		{
			float bx, by, bw, bh;
			lcdRect (t, bx, by, bw, bh);
			if (x >= bx && x <= bx + bw && y >= by && y <= by + bh)
				return t;
		}
		return -1;
	}

	bool hitBypass (int x, int y) const
	{
		float bx, by, bw, bh;
		bypassRect (bx, by, bw, bh);
		return x >= bx && x <= bx + bw && y >= by && y <= by + bh;
	}
	void toggleBypass ()
	{
		double nv = byp->load () ? 0.0 : 1.0;
		ctrl->beginEdit (kBypass);
		ctrl->setParamNormalized (kBypass, nv);
		ctrl->performEdit (kBypass, nv);
		ctrl->endEdit (kBypass);
		dirty = true;
	}

	// "i" icons next to the RNNoise headings: hover shows an explanation. [0] voice gate, [1] grace
	int infoCount () const { return rnnMode () ? 2 : 1; }
	float headWidth (const wchar_t* t) const
	{
		DWRITE_TEXT_METRICS m {};
		const_cast<SpeexView*> (this)->layoutFor (t, 13, DWRITE_FONT_WEIGHT_SEMI_BOLD, kLeft, 2.2f)->GetMetrics (&m);
		return m.widthIncludingTrailingWhitespace - 2.2f;
	}
	void infoPos (int i, float& x, float& y) const
	{
		if (!rnnMode ()) // Speex: next to the SUPPRESSION heading
		{
			x = 28.f + headWidth (L"SUPPRESSION") + 14.f;
			y = 298.f + 9.4f;
			return;
		}
		x = trL () + headWidth (i == 0 ? L"VOICE GATE" : L"GRACE") + 14.f;
		y = rowY (i == 0 ? 0 : 1) - 33.f + 9.4f;
	}
	const wchar_t* infoText (int i) const
	{
		if (!rnnMode ())
			return L"Maximum noise reduction (Speex). Lower value = stronger. Very quiet noise is removed even at 0 dB.";
		return i == 0 ? L"Mutes the mic when the model hears no voice (0% = off). Raise it until background noise is cut between words."
		              : L"How long the gate stays open after the voice stops, so word endings are not cut.";
	}
	int hitInfoIdx (int x, int y) const
	{
		if (tab != 0)
			return -1;
		for (int i = 0; i < infoCount (); ++i)
		{
			float ix, iy;
			infoPos (i, ix, iy);
			if ((x - ix) * (x - ix) + (y - iy) * (y - iy) <= 11.f * 11.f)
				return i;
		}
		return -1;
	}

	// --- inline numeric entry in a value box
	void startEdit (int target)
	{
		editing = true;
		editTarget = target;
		editFresh = true; // first typed character replaces the whole value
		editBuf = std::to_wstring (quant (target, normFor (target)));
		caretT = GetTickCount ();
		SetFocus (hwnd);
		dirty = true;
	}
	void commitEdit ()
	{
		if (!editing)
			return;
		editing = false;
		dirty = true;
		if (editBuf.empty () || editBuf == L"-")
			return;
		long v = std::wcstol (editBuf.c_str (), nullptr, 10);
		const int tg = editTarget;
		double t;
		if (tg == 1)
			t = std::clamp<long> (v, 0, 500) / 500.0; // grace period in ms
		else if (rnnMode ())
			t = std::clamp<long> (v, 0, 100) / 100.0; // percent
		else
		{
			if (v > 0)
				v = -v; // the range is 0..-60, so "30" means -30
			t = (std::clamp<long> (v, static_cast<long> (kMinDb), static_cast<long> (kMaxDb)) - kMinDb) / (kMaxDb - kMinDb);
		}
		const ParamID pid = tg == 1 ? kVadGrace : mainParam ();
		ctrl->beginEdit (pid);
		setNorm (t, tg);
		ctrl->endEdit (pid);
	}
	void cancelEdit ()
	{
		editing = false;
		dirty = true;
	}
	void editChar (wchar_t ch)
	{
		if (ch == 8) // backspace
		{
			if (editFresh)
				editBuf.clear ();
			else if (!editBuf.empty ())
				editBuf.pop_back ();
		}
		else if ((ch >= L'0' && ch <= L'9') || (ch == L'-' && !rnnMode ()))
		{
			if (editFresh)
				editBuf.clear ();
			if (ch == L'-' && !editBuf.empty ())
				return;
			if (editBuf.size () < 3)
				editBuf += ch;
		}
		else
			return;
		editFresh = false;
		caretT = GetTickCount ();
		dirty = true;
	}

	// RNNoise only works at 48 kHz
	bool rnnoiseOk () const { return std::fabs (rate->load () - 48000.0) < 1.0; }
	void chooseMethod (int m)
	{
		ctrl->beginEdit (kMethod);
		ctrl->setParamNormalized (kMethod, m / 2.0);
		ctrl->performEdit (kMethod, m / 2.0);
		ctrl->endEdit (kMethod);
		dirty = true;
	}

	bool hitPill (int x, int y) const
	{
		return tab == 0 && x >= kPillX && x <= kPillX + kPillW && y >= kPillY && y <= kPillY + kPillH;
	}
	// -1 = none, 0..2 = method row
	int hitMenuRow (int x, int y) const
	{
		if (x < kMenuX || x > kMenuX + kMenuW || y < kMenuY + 6 || y > kMenuY + 6 + 3 * kRowH)
			return -1;
		return static_cast<int> ((y - kMenuY - 6) / kRowH);
	}

	int hitTab (int x, int y) const
	{
		if (y < 12 || y > 46 || x < kTabX0 || x > kTabX1)
			return -1;
		return x < kTabMid ? 0 : 1;
	}

	//-------------------------------------------------------------- D2D resources
	void pickFont ()
	{
		fontFamily = L"Segoe UI";
		ComPtr<IDWriteFontCollection> coll;
		UINT32 idx;
		BOOL ok = FALSE;
		if (SUCCEEDED (dw->GetSystemFontCollection (&coll, FALSE)) &&
		    SUCCEEDED (coll->FindFamilyName (L"Segoe UI Variable Display", &idx, &ok)) && ok)
			fontFamily = L"Segoe UI Variable Display";
	}

	// Fredoka Regular is embedded as a resource (RCDATA 101) and registered privately for this window
	void loadBrandFont ()
	{
		if (FAILED (dw.As (&dw5)))
			return;
		HMODULE mod = nullptr;
		if (!GetModuleHandleExW (GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
		                         reinterpret_cast<LPCWSTR> (&SpeexView::registerClass), &mod))
			return;
		HRSRC res = FindResourceW (mod, MAKEINTRESOURCEW (101), RT_RCDATA);
		if (!res)
			return;
		const void* data = LockResource (LoadResource (mod, res));
		DWORD size = SizeofResource (mod, res);
		ComPtr<IDWriteFontFile> file;
		ComPtr<IDWriteFontSetBuilder1> builder;
		ComPtr<IDWriteFontSet> set;
		if (!data || FAILED (dw5->CreateInMemoryFontFileLoader (&memLoader)) ||
		    FAILED (dw5->RegisterFontFileLoader (memLoader.Get ())) ||
		    FAILED (memLoader->CreateInMemoryFontFileReference (dw5.Get (), data, size, nullptr, &file)) ||
		    FAILED (dw5->CreateFontSetBuilder (&builder)) || FAILED (builder->AddFontFile (file.Get ())) ||
		    FAILED (builder->CreateFontSet (&set)) || FAILED (dw5->CreateFontCollectionFromFontSet (set.Get (), &brandColl)))
		{
			brandColl.Reset (); // falls back to the system UI font
			return;
		}
		brandFamily.clear ();
		ComPtr<IDWriteFontFamily> fam;
		ComPtr<IDWriteLocalizedStrings> names;
		if (brandColl->GetFontFamilyCount () > 0 && SUCCEEDED (brandColl->GetFontFamily (0, &fam)) &&
		    SUCCEEDED (fam->GetFamilyNames (&names)) && names->GetCount () > 0)
		{
			UINT32 len = 0;
			if (SUCCEEDED (names->GetStringLength (0, &len)))
			{
				brandFamily.resize (len + 1);
				if (FAILED (names->GetString (0, &brandFamily[0], len + 1)))
					brandFamily.clear ();
				else
					brandFamily.resize (len);
			}
		}
		if (brandFamily.empty ())
			brandColl.Reset ();
	}

	void releaseTarget ()
	{
		for (auto& g : grad)
			g.Reset ();
		solid.Reset ();
		staticBmp.Reset ();
		staticRT.Reset ();
		staticTab = -1;
		cur = nullptr;
		rt.Reset ();
	}

	bool ensureTarget ()
	{
		if (rt)
			return true;
		D2D1_RENDER_TARGET_PROPERTIES rp = D2D1::RenderTargetProperties ();
		rp.dpiX = rp.dpiY = 96.f;
		if (FAILED (d2d->CreateHwndRenderTarget (
		        rp, D2D1::HwndRenderTargetProperties (hwnd, D2D1::SizeU (kW, kH), D2D1_PRESENT_OPTIONS_IMMEDIATELY),
		        &rt)))
			return false;
		rt->CreateSolidColorBrush (cWhite, &solid);
		const struct { Clr a, b; } defs[kGrads] = {
		    {C (50, 50, 53), C (28, 28, 30)},       // body
		    {C (70, 70, 74), C (42, 42, 45)},       // head
		    {C (255, 255, 255, 120), C (255, 255, 255, 6)}, // spectrum fill
		    {C (252, 252, 254), C (160, 160, 168)}, // slider fill
		    {C (240, 240, 244), C (128, 128, 136)}, // knob
		    {C (255, 255, 255), C (160, 160, 168)}, // knob hot
		    {C (240, 240, 243), C (196, 196, 202)}, // active tab pill
		    {C (4, 4, 5), C (16, 16, 18)},          // LCD
		};
		for (int i = 0; i < kGrads; ++i)
		{
			D2D1_GRADIENT_STOP st[2] = {{0.f, defs[i].a}, {1.f, defs[i].b}};
			ComPtr<ID2D1GradientStopCollection> sc;
			rt->CreateGradientStopCollection (st, 2, &sc);
			rt->CreateLinearGradientBrush (D2D1::LinearGradientBrushProperties (D2D1::Point2F (), D2D1::Point2F ()),
			                               sc.Get (), &grad[i]);
		}
		return true;
	}

	ID2D1LinearGradientBrush* vgrad (Grad g, float y0, float y1)
	{
		grad[g]->SetStartPoint (D2D1::Point2F (0, y0));
		grad[g]->SetEndPoint (D2D1::Point2F (0, y1));
		return grad[g].Get ();
	}

	//-------------------------------------------------------------- drawing primitives
	static D2D1_ROUNDED_RECT RR (float x, float y, float w, float h, float r)
	{
		return D2D1::RoundedRect (D2D1::RectF (x, y, x + w, y + h), r, r);
	}
	void fillRect (float x, float y, float w, float h, Clr c)
	{
		solid->SetColor (c);
		cur->FillRectangle (D2D1::RectF (x, y, x + w, y + h), solid.Get ());
	}
	void fillRR (float x, float y, float w, float h, float r, Clr c)
	{
		solid->SetColor (c);
		cur->FillRoundedRectangle (RR (x, y, w, h, r), solid.Get ());
	}
	void strokeRR (float x, float y, float w, float h, float r, Clr c, float wd = 1.f)
	{
		solid->SetColor (c);
		cur->DrawRoundedRectangle (RR (x, y, w, h, r), solid.Get (), wd);
	}
	void line (float x0, float y0, float x1, float y1, Clr c, float wd = 1.f)
	{
		solid->SetColor (c);
		cur->DrawLine (D2D1::Point2F (x0, y0), D2D1::Point2F (x1, y1), solid.Get (), wd);
	}

	IDWriteTextLayout* layoutFor (const wchar_t* s, float size, DWRITE_FONT_WEIGHT w, Al al, float track, bool brand = false)
	{
		brand = brand && brandColl;
		std::wstring key = s;
		key += brand ? L'\x2' : L'\x1';
		key += std::to_wstring (static_cast<int> (size * 10)) + L'/' + std::to_wstring (w) + L'/' +
		       std::to_wstring (al) + L'/' + std::to_wstring (static_cast<int> (track * 10));
		auto it = layouts.find (key);
		if (it != layouts.end ())
			return it->second.Get ();
		if (layouts.size () > 500)
			layouts.clear ();
		int fk = static_cast<int> (size * 10) * 1000 + w + al + (brand ? 900000000 : 0);
		auto& fmt = formats[fk];
		if (!fmt)
		{
			dw->CreateTextFormat (brand ? brandFamily.c_str () : fontFamily.c_str (), brand ? brandColl.Get () : nullptr, w,
			                      DWRITE_FONT_STYLE_NORMAL, DWRITE_FONT_STRETCH_NORMAL, size, L"en-us", &fmt);
			if (!fmt) // embedded font unavailable: fall back to the system UI font
				dw->CreateTextFormat (fontFamily.c_str (), nullptr, w, DWRITE_FONT_STYLE_NORMAL, DWRITE_FONT_STRETCH_NORMAL,
				                      size, L"en-us", &fmt);
			fmt->SetTextAlignment (al == kLeft ? DWRITE_TEXT_ALIGNMENT_LEADING
			                                   : (al == kCenter ? DWRITE_TEXT_ALIGNMENT_CENTER
			                                                    : DWRITE_TEXT_ALIGNMENT_TRAILING));
			fmt->SetWordWrapping (DWRITE_WORD_WRAPPING_NO_WRAP);
		}
		ComPtr<IDWriteTextLayout> lay;
		UINT32 len = static_cast<UINT32> (wcslen (s));
		dw->CreateTextLayout (s, len, fmt.Get (), 400.f, 60.f, &lay);
		if (track > 0.f)
		{
			ComPtr<IDWriteTextLayout1> l1;
			if (SUCCEEDED (lay.As (&l1)))
				l1->SetCharacterSpacing (0.f, track, 0.f, DWRITE_TEXT_RANGE {0, len});
		}
		layouts[key] = lay;
		return lay.Get ();
	}

	// y is the top of the text line box; for kCenter/kRight x is the center / right edge.
	void text (const wchar_t* s, float size, DWRITE_FONT_WEIGHT w, Clr c, float x, float y, Al al = kLeft,
	           float track = 0.f, bool brand = false)
	{
		IDWriteTextLayout* l = layoutFor (s, size, w, al, track, brand);
		float ox = al == kLeft ? x : (al == kCenter ? x - 200.f : x - 400.f);
		solid->SetColor (c);
		cur->DrawTextLayout (D2D1::Point2F (ox, y), l, solid.Get (), D2D1_DRAW_TEXT_OPTIONS_NONE);
	}

	void spectrumGeometry (const float* d, bool closed, ID2D1PathGeometry** out)
	{
		D2D1_POINT_2F p[Analyzer::kBands];
		for (int b = 0; b < Analyzer::kBands; ++b)
		{
			float v = std::clamp ((d[b] - kDbBot) / (kDbTop - kDbBot), 0.f, 1.f);
			p[b] = D2D1::Point2F (kPlotL + (kPlotR - kPlotL) * (b + 0.5f) / Analyzer::kBands, kPlotB - v * (kPlotB - kPlotT));
		}
		d2d->CreatePathGeometry (out);
		ComPtr<ID2D1GeometrySink> sink;
		(*out)->Open (&sink);
		if (closed)
		{
			sink->BeginFigure (D2D1::Point2F (p[0].x, kPlotB), D2D1_FIGURE_BEGIN_FILLED);
			sink->AddLine (p[0]);
		}
		else
			sink->BeginFigure (p[0], D2D1_FIGURE_BEGIN_HOLLOW);
		const int n = Analyzer::kBands;
		for (int i = 0; i < n - 1; ++i)
		{
			const D2D1_POINT_2F &p0 = p[std::max (i - 1, 0)], &p1 = p[i], &p2 = p[i + 1], &p3 = p[std::min (i + 2, n - 1)];
			sink->AddBezier (D2D1::BezierSegment (D2D1::Point2F (p1.x + (p2.x - p0.x) / 6.f, p1.y + (p2.y - p0.y) / 6.f),
			                                      D2D1::Point2F (p2.x - (p3.x - p1.x) / 6.f, p2.y - (p3.y - p1.y) / 6.f), p2));
		}
		if (closed)
		{
			sink->AddLine (D2D1::Point2F (p[n - 1].x, kPlotB));
			sink->EndFigure (D2D1_FIGURE_END_CLOSED);
		}
		else
			sink->EndFigure (D2D1_FIGURE_END_OPEN);
		sink->Close ();
	}

	void drawTrace (const float* d, bool fill, Clr col)
	{
		ComPtr<ID2D1PathGeometry> g;
		if (fill)
		{
			spectrumGeometry (d, true, &g);
			cur->FillGeometry (g.Get (), vgrad (gFill, kPlotT, kPlotB));
			g.Reset ();
		}
		spectrumGeometry (d, false, &g);
		if (fill)
		{
			solid->SetColor (C (255, 255, 255, 38));
			cur->DrawGeometry (g.Get (), solid.Get (), 4.f); // soft glow
		}
		solid->SetColor (col);
		cur->DrawGeometry (g.Get (), solid.Get (), 1.6f);
	}

	//-------------------------------------------------------------- scene
	void drawChrome ()
	{
		cur->FillRectangle (D2D1::RectF (0, 0, kW, kH), vgrad (gBody, 0, kH));

		// header
		cur->FillRectangle (D2D1::RectF (0, 0, kW, 58), vgrad (gHead, 0, 58));
		line (0, 0.5f, kW, 0.5f, C (255, 255, 255, 70));
		line (0, 58.5f, kW, 58.5f, C (0, 0, 0, 200));
		line (0, 59.5f, kW, 59.5f, C (255, 255, 255, 22));

		// logo (left) followed by "Noise", same rounded font
		strokeRR (13, 13.7f, 30.6f, 30.6f, 7.65f, C (205, 205, 210), 1.6f);
		text (L"el", 23, DWRITE_FONT_WEIGHT_NORMAL, C (225, 225, 230), 28.3f, 15.0f, kCenter, 0.f, true);
		text (L"Noise", 26.4f, DWRITE_FONT_WEIGHT_NORMAL, C (240, 240, 243), 47.9f, 12.4f, kLeft, 0.25f, true);

		if (tab == 0) // the engine selector only exists on the Main tab
		{
			// method selector (center): click to choose the engine
			fillRR (kPillX, kPillY, kPillW, kPillH, kPillH / 2, C (20, 20, 22));
			strokeRR (kPillX + 0.5f, kPillY + 0.5f, kPillW - 1, kPillH - 1, kPillH / 2 - 0.5f, C (84, 84, 90, 170));
			// label and value on one row (same center line), the pair centered in the pill
			{
				DWRITE_TEXT_METRICS m1 {}, m2 {};
				layoutFor (L"METHOD", 8, DWRITE_FONT_WEIGHT_SEMI_BOLD, kLeft, 1.4f)->GetMetrics (&m1);
				layoutFor (kMethodNames[meth->load ()], 13, DWRITE_FONT_WEIGHT_SEMI_BOLD, kLeft, 0.9f)->GetMetrics (&m2);
				const float gap = 12.f, w1 = m1.widthIncludingTrailingWhitespace - 1.4f, w2 = m2.widthIncludingTrailingWhitespace - 0.9f;
				const float x0 = kPillX + (kPillW - 24.f) / 2.f - (w1 + gap + w2) / 2.f;
				text (L"METHOD", 8, DWRITE_FONT_WEIGHT_SEMI_BOLD, C (100, 100, 106), x0, kPillY + kMethodLabelDy, kLeft, 1.4f);
				text (kMethodNames[meth->load ()], 13, DWRITE_FONT_WEIGHT_SEMI_BOLD, cWhite, x0 + w1 + gap, kPillY + kMethodValueDy, kLeft, 0.9f);
			}
			{
				const float ax = kPillX + kPillW - 20, ay = kPillY + kPillH / 2;
				line (ax - 4, ay - 1.5f, ax, ay + 2.5f, cText, 1.6f);
				line (ax, ay + 2.5f, ax + 4, ay - 1.5f, cText, 1.6f);
			}
		}

		// segmented tabs
		fillRR (kTabX0, 16, kTabX1 - kTabX0, 26, 13, C (20, 20, 22));
		strokeRR (kTabX0 + 0.5f, 16.5f, kTabX1 - kTabX0 - 1, 25, 12.5f, C (84, 84, 90, 160));
		float ax = tab == 0 ? kTabX0 + 2 : kTabMid, aw = tab == 0 ? kTabMid - kTabX0 - 2 : kTabX1 - kTabMid - 2;
		cur->FillRoundedRectangle (RR (ax, 18, aw, 22, 11), vgrad (gPill, 18, 40));
		text (L"MAIN", 11, DWRITE_FONT_WEIGHT_SEMI_BOLD, tab == 0 ? C (24, 24, 26) : cMuted, (kTabX0 + kTabMid) / 2.f, 22,
		      kCenter, 1.5f);
		text (L"ABOUT", 11, DWRITE_FONT_WEIGHT_SEMI_BOLD, tab == 1 ? C (24, 24, 26) : cMuted, (kTabMid + kTabX1) / 2.f, 22,
		      kCenter, 1.5f);

		// screws
		for (float sx : {9.f, kW - 9.f})
		{
			float sy = kH - 9.f;
			solid->SetColor (C (22, 22, 24));
			cur->FillEllipse (D2D1::Ellipse (D2D1::Point2F (sx, sy), 4, 4), solid.Get ());
			solid->SetColor (C (88, 88, 94));
			cur->DrawEllipse (D2D1::Ellipse (D2D1::Point2F (sx, sy), 4, 4), solid.Get (), 1.f);
			line (sx - 2, sy + 2, sx + 2, sy - 2, C (88, 88, 94));
		}
		strokeRR (0.5f, 0.5f, kW - 1, kH - 1, 0, C (10, 10, 12));
	}

	void drawDisplay ()
	{
		// screen
		fillRR (12, kDispT, kW - 24, kDispB - kDispT, 8, C (0, 0, 0));
		strokeRR (12.5f, kDispT + 0.5f, kW - 25, kDispB - kDispT - 1, 8, C (22, 22, 25), 1.f);
		line (16, kDispB + 1.5f, kW - 16, kDispB + 1.5f, C (255, 255, 255, 26));
		line (16, kDispT + 1.5f, kW - 16, kDispT + 1.5f, C (255, 255, 255, 10));

		// legend
		line (28, 87, 46, 87, cLine, 1.6f);
		text (L"INPUT", 10, DWRITE_FONT_WEIGHT_SEMI_BOLD, C (150, 150, 156), 52, 80, kLeft, 1.2f);
		line (108, 87, 126, 87, cOut, 1.6f);
		text (L"OUTPUT", 10, DWRITE_FONT_WEIGHT_SEMI_BOLD, C (225, 225, 230), 132, 80, kLeft, 1.2f);
		if (!rnnMode ())
			text (L"SPECTRUM", 10, DWRITE_FONT_WEIGHT_SEMI_BOLD, C (96, 96, 102), kW - 28, 80, kRight, 1.5f);

		// grid
		const float fmax = std::min (Analyzer::kFMax, 0.45f * 48000.f);
		auto fx = [&] (float f) {
			return kPlotL + (kPlotR - kPlotL) * std::log (f / Analyzer::kFMin) / std::log (fmax / Analyzer::kFMin);
		};
		for (int db = -20; db > -100; db -= 20)
		{
			float y = std::floor (kPlotB - (db - kDbBot) / (kDbTop - kDbBot) * (kPlotB - kPlotT)) + 0.5f;
			line (kPlotL, y, kPlotR, y, C (255, 255, 255, 14));
			wchar_t b[8];
			swprintf (b, 8, L"%d", db);
			text (b, 9, DWRITE_FONT_WEIGHT_NORMAL, C (80, 80, 86), kPlotL - 22, y - 7, kLeft);
		}
		const struct { float f; const wchar_t* t; } marks[] = {{100, L"100"}, {200, L"200"}, {500, L"500"},
		                                                        {1000, L"1K"}, {2000, L"2K"}, {5000, L"5K"},
		                                                        {10000, L"10K"}};
		for (auto& mk : marks)
		{
			float x = std::floor (fx (mk.f)) + 0.5f;
			line (x, kPlotT, x, kPlotB, C (255, 255, 255, 12));
			text (mk.t, 10, DWRITE_FONT_WEIGHT_NORMAL, C (106, 106, 112), x, kPlotB + 5, kCenter);
		}
		line (kPlotL, kPlotB + 0.5f, kPlotR, kPlotB + 0.5f, C (255, 255, 255, 40));

		const float cx[3] = {kW * 0.2f, kW * 0.5f, kW * 0.8f};
		const wchar_t* lab[3] = {L"INPUT", L"REDUCTION", L"OUTPUT"};
		for (int i = 0; i < 3; ++i)
			text (lab[i], 9, DWRITE_FONT_WEIGHT_SEMI_BOLD, C (100, 100, 106), cx[i], 240, kCenter, 1.5f);
		line (kW * 0.35f, 244, kW * 0.35f, 266, C (255, 255, 255, 18));
		line (kW * 0.65f, 244, kW * 0.65f, 266, C (255, 255, 255, 18));
	}

	void drawDisplayDynamic ()
	{
		drawTrace (sIn, false, cLine);
		drawTrace (sOut, true, cOut);

		wchar_t b[32];
		const float cx[3] = {kW * 0.2f, kW * 0.5f, kW * 0.8f};
		float red = std::max (0.f, lvIn - lvOut);
		float val[3] = {lvIn, red, lvOut};
		for (int i = 0; i < 3; ++i)
		{
			if (i != 1 && val[i] < -90.f)
				wcscpy_s (b, L"-inf dB");
			else
				swprintf (b, 32, L"%.1f dB", val[i]);
			text (b, 15, DWRITE_FONT_WEIGHT_NORMAL, i == 0 ? C (176, 176, 182) : cWhite, cx[i], 250, kCenter);
		}
	}

	void lcdFrame (float bx, float by, float bw, float bh)
	{
		cur->FillRoundedRectangle (RR (bx, by, bw, bh, 8), vgrad (gLcd, by, by + bh));
		strokeRR (bx + 0.5f, by + 0.5f, bw - 1, bh - 1, 8, C (0, 0, 0), 1.f);
		line (bx + 6, by + bh + 1.5f, bx + bw - 6, by + bh + 1.5f, C (255, 255, 255, 24));
		line (bx + 8, by + 1.5f, bx + bw - 8, by + 1.5f, C (255, 255, 255, 12));
	}
	void grooveFrame (float ry, float l, float r)
	{
		fillRR (l - 5, ry - 5, r - l + 10, 10, 5, C (6, 6, 7));
		strokeRR (l - 4.5f, ry - 4.5f, r - l + 9, 9, 4.5f, C (66, 66, 72), 1.f);
		line (l, ry - 3.5f, r, ry - 3.5f, C (0, 0, 0, 160));
	}

	void drawSlider ()
	{
		line (0, 283.5f, kW, 283.5f, C (0, 0, 0, 160));
		line (0, 284.5f, kW, 284.5f, C (255, 255, 255, 20));

		if (!rnnMode ())
		{
			// Speex: unchanged from v1.1
			text (L"SUPPRESSION", 13, DWRITE_FONT_WEIGHT_SEMI_BOLD, cText, 28, 298, kLeft, 2.2f);
			text (L"Lower value = stronger noise reduction", 11, DWRITE_FONT_WEIGHT_NORMAL, cMuted, 28, 320, kLeft);
			lcdFrame (kLcdX, kLcdY, kLcdW, kLcdH);
			text (L"dB", 13, DWRITE_FONT_WEIGHT_NORMAL, cMuted, kLcdX + kLcdW - 14, kLcdY + 21, kRight);
			grooveFrame (kTrackY, kTrackL, kTrackR);
			for (int i = 0; i <= 30; i += 5)
			{
				float x = std::floor (kTrackL + (kTrackR - kTrackL) * i / 30.f) + 0.5f;
				wchar_t lb[8];
				swprintf (lb, 8, L"%d", -60 + i * 2);
				text (lb, 10, DWRITE_FONT_WEIGHT_NORMAL, C (140, 140, 146), x, kTrackY + 28.f, kCenter);
			}
			return;
		}

		// RNNoise: two identical rows. target 1 = grace period (top), target 0 = voice gate (bottom)
		for (int tg = 1; tg >= 0; --tg)
		{
			const float ry = rowY (tg);
			text (tg == 1 ? L"GRACE" : L"VOICE GATE", 13, DWRITE_FONT_WEIGHT_SEMI_BOLD, cText, trL (), ry - 33.f, kLeft, 2.2f);
			float bx, by, bw, bh;
			lcdRect (tg, bx, by, bw, bh);
			lcdFrame (bx, by, bw, bh);
			grooveFrame (ry, trL (), trR ());
		}
		for (int i = 0; i <= 5; ++i) // tick labels under the voice gate slider
		{
			float x = std::floor (trL () + (trR () - trL ()) * i / 5.f) + 0.5f;
			wchar_t lb[8];
			swprintf (lb, 8, L"%d", i * 20);
			text (lb, 10, DWRITE_FONT_WEIGHT_NORMAL, C (140, 140, 146), x, rowY (0) + 23.f, kCenter);
		}
	}

	// capsule-shaped slider knob (same look for every slider)
	void drawCapsule (float cx, float cy, float w, float h, bool hot, int grips, float spacing = 3.5f)
	{
		const float r = w * 0.42f;
		for (int i = 4; i >= 1; --i)
			fillRR (cx - w / 2 - i, cy - h / 2 + i + 3, w + 2 * i, h - 2, r + i, C (0, 0, 0, 22));
		cur->FillRoundedRectangle (RR (cx - w / 2, cy - h / 2, w, h, r), vgrad (hot ? gKnobHot : gKnob, cy - h / 2, cy + h / 2));
		strokeRR (cx - w / 2 + 0.5f, cy - h / 2 + 0.5f, w - 1, h - 1, r - 0.5f, C (14, 14, 16), 1.f);
		line (cx - w / 2 + 5.f, cy - h / 2 + 2.f, cx + w / 2 - 5.f, cy - h / 2 + 2.f, C (255, 255, 255, 150));
		const float gh = h * 0.23f;
		for (int k = 0; k < grips; ++k)
		{
			const float gx = cx + (k - (grips - 1) / 2.f) * spacing;
			line (gx, cy - gh, gx, cy + gh, C (58, 58, 64), 1.5f);
		}
	}

	// number (or OFF) and unit inside a value box, including the typing state
	void drawLcdContent (int tg)
	{
		const bool rn = rnnMode ();
		float bx, by, bw, bh;
		lcdRect (tg, bx, by, bw, bh);
		const float rightOff = rn ? 40.f : 44.f, unitOff = rn ? 11.f : 14.f;
		const float numY = by + (bh - 40.f) / 2.f + 1.5f, unitY = by + bh / 2.f - 2.f;
		const double t = tg == 1 ? graceNorm () : curNorm ();
		const int v = quant (tg, t);
		wchar_t b[16];
		swprintf (b, 16, L"%d", v);
		const bool ed = editing && editTarget == tg;
		if (!ed && rn && tg == 0 && v == 0)
		{
			text (L"OFF", 26, DWRITE_FONT_WEIGHT_LIGHT, C (150, 150, 156), bx + bw / 2.f, by + (bh - 35.f) / 2.f, kCenter, 1.5f);
			return;
		}
		if (!ed)
		{
			text (b, 30, DWRITE_FONT_WEIGHT_LIGHT, cWhite, bx + bw - rightOff, numY, kRight);
			if (rn)
				text (tg == 1 ? L"ms" : L"%", 13, DWRITE_FONT_WEIGHT_NORMAL, cMuted, bx + bw - unitOff, unitY, kRight);
			return;
		}
		strokeRR (bx + 0.5f, by + 0.5f, bw - 1, bh - 1, 8, C (255, 255, 255, 150), 1.5f);
		const float right = bx + bw - rightOff;
		float w = 0.f;
		if (!editBuf.empty ())
		{
			IDWriteTextLayout* l = layoutFor (editBuf.c_str (), 30, DWRITE_FONT_WEIGHT_LIGHT, kRight, 0.f);
			DWRITE_TEXT_METRICS m {};
			l->GetMetrics (&m);
			w = m.widthIncludingTrailingWhitespace;
		}
		if (editFresh && w > 0.f)
			fillRect (right - w - 3, by + 7, w + 6, bh - 14, C (255, 255, 255, 60));
		if (!editBuf.empty ())
			text (editBuf.c_str (), 30, DWRITE_FONT_WEIGHT_LIGHT, cWhite, right, numY, kRight);
		if (((GetTickCount () - caretT) / 530) % 2 == 0 && !editFresh)
			line (right + 2, by + 9, right + 2, by + bh - 9, cWhite, 1.5f);
	}

	void drawInfoIcons ()
	{
		for (int i = 0; i < infoCount (); ++i)
		{
			float ix, iy;
			infoPos (i, ix, iy);
			const bool h = hoverInfo == i;
			solid->SetColor (h ? cWhite : C (130, 130, 136));
			cur->DrawEllipse (D2D1::Ellipse (D2D1::Point2F (ix, iy), 7.f, 7.f), solid.Get (), 1.3f);
			text (L"i", 10, DWRITE_FONT_WEIGHT_SEMI_BOLD, h ? cWhite : C (150, 150, 156), ix, iy - 6.3f, kCenter);
		}
	}

	void drawSliderDynamic ()
	{
		const bool rn = rnnMode ();
		// bypass pill (top right of the display in RNNoise mode, next to the value box in Speex mode)
		{
			float px, py, pw, ph;
			bypassRect (px, py, pw, ph);
			const bool on = byp->load ();
			if (on)
				rt->FillRoundedRectangle (RR (px, py, pw, ph, ph / 2), vgrad (gPill, py, py + ph));
			else
				fillRR (px, py, pw, ph, ph / 2, C (22, 22, 24));
			strokeRR (px + 0.5f, py + 0.5f, pw - 1, ph - 1, ph / 2 - 0.5f, on ? C (255, 255, 255, 90) : C (84, 84, 90, 170));
			solid->SetColor (on ? C (24, 24, 26) : C (96, 96, 102));
			rt->FillEllipse (D2D1::Ellipse (D2D1::Point2F (px + (rn ? 17.f : 15.f), py + ph / 2), rn ? 3.7f : 3.5f, rn ? 3.7f : 3.5f), solid.Get ());
			text (L"BYPASS", 10, DWRITE_FONT_WEIGHT_SEMI_BOLD, on ? C (24, 24, 26) : cMuted, px + 58.f,
			      py + (rn ? 7.2f : 6.5f), kCenter, 1.5f);
		}

		drawLcdContent (0);
		if (rn)
			drawLcdContent (1);

		if (!rn)
		{
			// Speex: unchanged from v1.1
			const double t = curNorm ();
			const float kx = static_cast<float> (kTrackL + t * (kTrackR - kTrackL));
			if (kx > kTrackL + 1)
			{
				fillRR (kTrackL - 3, kTrackY - 3, kx - kTrackL + 3, 6, 3, C (255, 255, 255, 34));
				cur->FillRoundedRectangle (RR (kTrackL - 3, kTrackY - 2.5f, kx - kTrackL + 3, 5, 2.5f), vgrad (gSlider, kTrackY - 3, kTrackY + 3));
			}
			for (int i = 0; i <= 30; ++i)
			{
				float x = std::floor (kTrackL + (kTrackR - kTrackL) * i / 30.f) + 0.5f;
				bool major = i % 5 == 0, lit = x <= kx + 0.5f;
				line (x, kTrackY + 16.f, x, kTrackY + (major ? 26.f : 21.f), lit ? C (220, 220, 225) : cDim);
			}
			drawCapsule (kx, kTrackY, 22.f, 40.f, (dragging && dragTarget == 0) || hoverSlider, 3, 4.f);
			drawInfoIcons ();
			return;
		}

		// RNNoise: two identical rows
		for (int tg = 1; tg >= 0; --tg)
		{
			const float ry = rowY (tg);
			const double t = tg == 1 ? graceNorm () : curNorm ();
			const float kx = static_cast<float> (trL () + t * (trR () - trL ()));
			if (tg == 1)
			{
				if (kx > trL () + 1)
				{
					fillRR (trL () - 3, ry - 3, kx - trL () + 3, 6, 3, C (255, 255, 255, 34));
					cur->FillRoundedRectangle (RR (trL () - 3, ry - 2.5f, kx - trL () + 3, 5, 2.5f), vgrad (gSlider, ry - 3, ry + 3));
				}
			}
			else
			{
				// live voice probability fills the groove; the knob is the gate threshold
				const float pw = std::clamp (vadShown, 0.f, 1.f) * (trR () - trL () + 6);
				if (pw > 2.f)
				{
					cur->FillRoundedRectangle (RR (trL () - 3, ry - 2.5f, pw, 5, 2.5f), vgrad (gSlider, ry - 3, ry + 3));
					fillRR (trL () - 3, ry - 3, pw, 6, 3, C (255, 255, 255, 30));
				}
				const bool voice = vadShown >= (t > 0.0 ? static_cast<float> (t) : 0.5f);
				const float sx = trR () + 4.f;
				text (voice ? L"VOICE" : L"NO VOICE", 10, DWRITE_FONT_WEIGHT_SEMI_BOLD, voice ? cWhite : cDim, sx, ry - 32.f, kRight, 1.4f);
				solid->SetColor (voice ? cWhite : cDim);
				cur->FillEllipse (D2D1::Ellipse (D2D1::Point2F (sx - (voice ? 56.f : 76.f), ry - 25.5f), 3.f, 3.f), solid.Get ());
			}
			const int nt = tg == 1 ? 10 : 20;
			for (int i = 0; i <= nt; ++i)
			{
				float x = std::floor (trL () + (trR () - trL ()) * i / float (nt)) + 0.5f;
				bool major = i % 5 == 0, lit = x <= kx + 0.5f;
				line (x, ry + 15.f, x, ry + (major ? 21.f : 18.f), lit ? C (220, 220, 225) : cDim);
			}
			drawCapsule (kx, ry, 18.f, 34.f, dragging && dragTarget == tg ? true : (tg == 0 && hoverSlider), 3);
		}

		drawInfoIcons ();
	}

	void drawAbout ()
	{
		strokeRR (kW / 2.f - 34, 96, 68, 68, 16, C (205, 205, 210), 3.f);
		text (L"el", 40, DWRITE_FONT_WEIGHT_NORMAL, C (225, 225, 230), kW / 2.f, 103, kCenter, 0.f, true);
		text (L"elNoise", 38, DWRITE_FONT_WEIGHT_NORMAL, cWhite, kW / 2.f, 174, kCenter, 0.5f, true);
		text (L"VST3  |  version 1.2.0", 13, DWRITE_FONT_WEIGHT_NORMAL, cMuted, kW / 2.f, 222, kCenter, 0.4f);
		line (kW / 2.f - 60, 262.5f, kW / 2.f + 60, 262.5f, C (255, 255, 255, 40));
		text (L"MADE BY", 11, DWRITE_FONT_WEIGHT_SEMI_BOLD, cMuted, kW / 2.f, 282, kCenter, 2.5f);
		text (L"elem3n7ary", 34, DWRITE_FONT_WEIGHT_NORMAL, cWhite, kW / 2.f, 295, kCenter, 0.5f, true);
		text (L"Based on SpeexDSP and RNNoise (Xiph.Org, BSD license)", 12, DWRITE_FONT_WEIGHT_NORMAL, cMuted, kW / 2.f, 380, kCenter);
	}

	void buildStatic ()
	{
		if (!staticRT)
		{
			if (FAILED (rt->CreateCompatibleRenderTarget (D2D1::SizeF (kW, kH), &staticRT)))
				return;
		}
		cur = staticRT.Get ();
		staticRT->BeginDraw ();
		staticRT->SetTransform (D2D1::Matrix3x2F::Identity ());
		drawChrome ();
		if (tab == 0)
		{
			drawDisplay ();
			drawSlider ();
		}
		else
			drawAbout ();
		staticRT->EndDraw ();
		staticBmp.Reset ();
		staticRT->GetBitmap (&staticBmp);
		cur = rt.Get ();
		staticTab = tab;
		staticMethod = meth->load ();
	}

	// small wrapped text bubble above (anchorX, anchorY)
	void drawTooltip (const wchar_t* txt, float anchorX, float anchorY)
	{
		ComPtr<IDWriteTextFormat> fmt;
		dw->CreateTextFormat (fontFamily.c_str (), nullptr, DWRITE_FONT_WEIGHT_NORMAL, DWRITE_FONT_STYLE_NORMAL,
		                      DWRITE_FONT_STRETCH_NORMAL, 12.f, L"en-us", &fmt);
		if (!fmt)
			return;
		fmt->SetWordWrapping (DWRITE_WORD_WRAPPING_WRAP);
		ComPtr<IDWriteTextLayout> lay;
		dw->CreateTextLayout (txt, static_cast<UINT32> (wcslen (txt)), fmt.Get (), 250.f, 200.f, &lay);
		if (!lay)
			return;
		DWRITE_TEXT_METRICS m {};
		lay->GetMetrics (&m);
		const float bw = m.width + 22.f, bh = m.height + 18.f;
		const float bx = std::clamp (anchorX - 20.f, 8.f, kW - bw - 8.f), by = anchorY - bh - 6.f;
		for (int i = 4; i >= 1; --i)
			fillRR (bx - i, by - i + 4, bw + 2 * i, bh + 2 * i, 10 + i, C (0, 0, 0, 30));
		fillRR (bx, by, bw, bh, 10, C (30, 30, 33));
		strokeRR (bx + 0.5f, by + 0.5f, bw - 1, bh - 1, 9.5f, C (96, 96, 102, 200), 1.f);
		solid->SetColor (cText);
		cur->DrawTextLayout (D2D1::Point2F (bx + 11.f, by + 9.f), lay.Get (), solid.Get (), D2D1_DRAW_TEXT_OPTIONS_NONE);
	}

	void drawMenu ()
	{
		const float h = 12 + 3 * kRowH;
		for (int i = 4; i >= 1; --i)
			fillRR (kMenuX - i, kMenuY - i + 4, kMenuW + 2 * i, h + 2 * i, 12 + i, C (0, 0, 0, 30));
		fillRR (kMenuX, kMenuY, kMenuW, h, 12, C (30, 30, 33));
		strokeRR (kMenuX + 0.5f, kMenuY + 0.5f, kMenuW - 1, h - 1, 11.5f, C (96, 96, 102, 200), 1.f);
		strokeRR (kPillX + 0.5f, kPillY + 0.5f, kPillW - 1, kPillH - 1, kPillH / 2 - 0.5f, C (255, 255, 255, 120), 1.f);
		const wchar_t* title[3] = {L"Speex", L"RNNoise", L"RNNoise Lite"};
		const bool ok = rnnoiseOk ();
		const wchar_t* sub[3] = {L"Light on CPU, classic suppression",
		                         ok ? L"Best quality, uses more CPU" : L"Needs a 48 kHz session",
		                         ok ? L"Nearly as good, lighter on CPU" : L"Needs a 48 kHz session"};
		for (int i = 0; i < 3; ++i)
		{
			const float y = kMenuY + 6 + i * kRowH;
			const bool enabled = i == 0 || ok;
			if (enabled && hoverRow == i)
				fillRR (kMenuX + 6, y + 2, kMenuW - 12, kRowH - 4, 8, C (255, 255, 255, 22));
			text (title[i], 14, DWRITE_FONT_WEIGHT_SEMI_BOLD, enabled ? cWhite : C (110, 110, 116), kMenuX + 18, y + 6, kLeft, 0.3f);
			text (sub[i], 11, DWRITE_FONT_WEIGHT_NORMAL, enabled ? cMuted : C (84, 84, 90), kMenuX + 18, y + 27, kLeft);
			if (i == meth->load ())
			{
				const float cx = kMenuX + kMenuW - 22, cy = y + kRowH / 2;
				line (cx - 4, cy, cx - 1, cy + 3, cWhite, 1.8f);
				line (cx - 1, cy + 3, cx + 5, cy - 4, cWhite, 1.8f);
			}
		}
	}

	void render ()
	{
		if (!ensureTarget ())
			return;
		if (staticTab != tab || staticMethod != meth->load () || !staticBmp)
			buildStatic ();
		cur = rt.Get ();
		rt->BeginDraw ();
		rt->SetTransform (D2D1::Matrix3x2F::Identity ());
		if (staticBmp)
			rt->DrawBitmap (staticBmp.Get (), D2D1::RectF (0, 0, kW, kH), 1.f, D2D1_BITMAP_INTERPOLATION_MODE_NEAREST_NEIGHBOR);
		if (tab == 0)
		{
			drawDisplayDynamic ();
			drawSliderDynamic ();
		}
		if (tab == 0 && hoverInfo >= 0 && hoverInfo < infoCount () && !menuOpen)
		{
			float ix, iy;
			infoPos (hoverInfo, ix, iy);
			drawTooltip (infoText (hoverInfo), ix, iy - 14.f);
		}
		if (menuOpen)
			drawMenu ();
		if (rt->EndDraw () == D2DERR_RECREATE_TARGET)
		{
			releaseTarget ();
			dirty = true;
		}
		std::copy (sIn, sIn + Analyzer::kBands, rIn);
		std::copy (sOut, sOut + Analyzer::kBands, rOut);
		rLvIn = lvIn;
		rLvOut = lvOut;
		rVad = vadShown;
	}

	// returns true when the spectrum / levels moved enough to be worth a redraw
	bool updateData ()
	{
		auto follow = [] (float* d, float* s, const Analyzer* a)
		{
			const int n = Analyzer::kBands;
			for (int b = 0; b < n; ++b)
			{
				float v = a->get (b);
				d[b] = v > d[b] ? v : d[b] * 0.9f + v * 0.1f;
			}
			for (int b = 1; b < n - 1; ++b)
				s[b] = 0.25f * d[b - 1] + 0.5f * d[b] + 0.25f * d[b + 1];
			s[0] = d[0];
			s[n - 1] = d[n - 1];
		};
		follow (dIn, sIn, aIn);
		follow (dOut, sOut, aOut);
		auto lvl = [] (float& cur, float v) { cur = v > cur ? cur * 0.6f + v * 0.4f : cur * 0.94f + v * 0.06f; };
		lvl (lvIn, aIn->level ());
		lvl (lvOut, aOut->level ());

		vadShown = std::max (vprob->load (), vadShown * 0.88f);
		float diff = std::fabs (lvIn - rLvIn) + std::fabs (lvOut - rLvOut) + (rnnMode () ? 40.f * std::fabs (vadShown - rVad) : 0.f);
		for (int b = 0; b < Analyzer::kBands; ++b)
			diff = std::max (diff, std::max (std::fabs (sIn[b] - rIn[b]), std::fabs (sOut[b] - rOut[b])));
		return diff > 0.04f;
	}

	void tick ()
	{
		pending = false;
		if (!hwnd || !IsWindowVisible (hwnd) || IsIconic (GetAncestor (hwnd, GA_ROOT)))
			return;
		if (rt && (rt->CheckWindowState () & D2D1_WINDOW_STATE_OCCLUDED))
			return;
		bool changed = tab == 0 && updateData ();
		if (editing && ((GetTickCount () - caretT) / 530) != caretPhase)
		{
			caretPhase = (GetTickCount () - caretT) / 530;
			changed = true;
		}
		double cur = dbValue->load (), thr = vthr->load (), gr = vgrace->load ();
		if (std::fabs (cur - lastDb) > 1e-6 || std::fabs (thr - lastThr) > 1e-6 || std::fabs (gr - lastGrace) > 1e-6)
		{
			lastDb = cur;
			lastThr = thr;
			lastGrace = gr;
			changed = true;
		}
		// the analyzer only needs 30 fps; interaction (dirty / dragging) gets the full display rate
		if (!dirty && !dragging && (++skip & 1))
			return;
		if (dirty || changed)
		{
			dirty = false;
			render ();
		}
	}

	static LRESULT CALLBACK wndProc (HWND h, UINT m, WPARAM wp, LPARAM lp)
	{
		auto* self = reinterpret_cast<SpeexView*> (GetWindowLongPtrW (h, GWLP_USERDATA));
		int x = static_cast<short> (LOWORD (lp)), y = static_cast<short> (HIWORD (lp));
		if (m == kTickMsg)
		{
			if (self)
				self->tick ();
			return 0;
		}
		switch (m)
		{
			case WM_CREATE:
				self = static_cast<SpeexView*> (reinterpret_cast<CREATESTRUCTW*> (lp)->lpCreateParams);
				SetWindowLongPtrW (h, GWLP_USERDATA, reinterpret_cast<LONG_PTR> (self));
				return 0;
			case WM_ERASEBKGND:
				return 1;
			case WM_PAINT:
			{
				PAINTSTRUCT ps;
				BeginPaint (h, &ps);
				EndPaint (h, &ps);
				if (self)
				{
					self->dirty = true;
					self->render (); // expose: draw right away so there is no blank frame
					self->dirty = false;
				}
				return 0;
			}
			case WM_LBUTTONDOWN:
			case WM_LBUTTONDBLCLK:
			{
				if (!self)
					return 0;
				if (self->menuOpen)
				{
					int row = self->hitMenuRow (x, y);
					const bool usable = row == 0 || (row > 0 && self->rnnoiseOk ());
					if (usable)
						self->chooseMethod (row);
					if (usable || row < 0) // a disabled row keeps the menu open
					{
						self->menuOpen = false;
						self->dirty = true;
					}
					return 0;
				}
				if (self->hitPill (x, y))
				{
					if (self->editing)
						self->commitEdit ();
					self->menuOpen = true;
					self->hoverRow = -1;
					self->dirty = true;
					return 0;
				}
				const int lcd = self->hitLcdIdx (x, y);
				if (self->editing && lcd != self->editTarget)
					self->commitEdit ();
				if (lcd >= 0)
				{
					if (!self->editing)
						self->startEdit (lcd);
					return 0;
				}
				if (self->tab == 0 && self->hitBypass (x, y))
				{
					self->toggleBypass ();
					return 0;
				}
				int t = self->hitTab (x, y);
				if (t >= 0)
				{
					self->tab = t;
					self->menuOpen = false;
					self->dirty = true;
				}
				else if (self->tab == 0 && (self->onSlider (x, y) || self->onGrace (x, y)))
				{
					self->dragTarget = self->onGrace (x, y) ? 1 : 0;
					self->dragging = true;
					self->dragNorm = self->normFor (self->dragTarget);
					self->lastSentDb = self->quant (self->dragTarget, self->dragNorm);
					self->lastX = x;
					SetCapture (h);
					self->ctrl->beginEdit (self->dragParam ());
					if (!(GetKeyState (VK_SHIFT) & 0x8000))
						self->setNorm ((x - self->trL ()) / double (self->trR () - self->trL ()));
				}
				return 0;
			}
			case WM_MOUSEMOVE:
				if (!self)
					return 0;
				if (self->dragging)
				{
					const double l = self->trL (), w = self->trR () - self->trL ();
					if (GetKeyState (VK_SHIFT) & 0x8000) // fine adjust: 5x slower, relative
						self->setNorm (self->dragNorm + (x - self->lastX) / (5.0 * w));
					else
						self->setNorm ((x - l) / w);
					self->lastX = x;
				}
				else if (self->menuOpen)
				{
					int row = self->hitMenuRow (x, y);
					if (row != self->hoverRow)
					{
						self->hoverRow = row;
						self->dirty = true;
					}
				}
				else
				{
					bool hv = self->tab == 0 && self->onSlider (x, y);
					if (hv != self->hoverSlider)
					{
						self->hoverSlider = hv;
						self->dirty = true;
					}
					int hi = self->hitInfoIdx (x, y);
					if (hi != self->hoverInfo)
					{
						self->hoverInfo = hi;
						self->dirty = true;
					}
					TRACKMOUSEEVENT te {sizeof (te), TME_LEAVE, h, 0};
					TrackMouseEvent (&te);
				}
				return 0;
			case WM_MOUSELEAVE:
				if (self && (self->hoverSlider || self->hoverInfo >= 0))
				{
					self->hoverSlider = false;
					self->hoverInfo = -1;
					self->dirty = true;
				}
				return 0;
			case WM_GETDLGCODE:
				return DLGC_WANTALLKEYS | DLGC_WANTCHARS;
			case WM_CHAR:
				if (self && self->editing)
				{
					if (wp == 13)
						self->commitEdit ();
					else if (wp == 27)
						self->cancelEdit ();
					else
						self->editChar (static_cast<wchar_t> (wp));
				}
				return 0;
			case WM_KEYDOWN:
				if (self && self->editing)
				{
					if (wp == VK_RETURN)
						self->commitEdit ();
					else if (wp == VK_ESCAPE)
						self->cancelEdit ();
					return 0;
				}
				break;
			case WM_KILLFOCUS:
				if (self && self->editing)
					self->commitEdit ();
				return 0;
			case WM_SETCURSOR:
			{
				POINT pt;
				GetCursorPos (&pt);
				ScreenToClient (h, &pt);
				bool hand = self && (self->hitTab (pt.x, pt.y) >= 0 || (self->tab == 0 && (self->dragging || self->onSlider (pt.x, pt.y) || self->onGrace (pt.x, pt.y))));
				if (self && (self->hitPill (pt.x, pt.y) || (self->menuOpen && (self->hitMenuRow (pt.x, pt.y) == 0 || (self->hitMenuRow (pt.x, pt.y) > 0 && self->rnnoiseOk ())))))
					hand = true;
				bool ibeam = self && !self->menuOpen && self->hitLcdIdx (pt.x, pt.y) >= 0;
				if (self && self->tab == 0 && self->hitBypass (pt.x, pt.y))
					hand = true;
				SetCursor (LoadCursor (nullptr, ibeam ? IDC_IBEAM : (hand ? IDC_HAND : IDC_ARROW)));
				return TRUE;
			}
			case WM_LBUTTONUP:
				if (self && self->dragging)
				{
					self->dragging = false;
					ReleaseCapture ();
					const ParamID pid = self->dragParam ();
					self->ctrl->setParamNormalized (pid, self->dragNorm);
					self->ctrl->performEdit (pid, self->dragNorm);
					self->ctrl->endEdit (pid);
					self->dirty = true;
				}
				return 0;
			case WM_MOUSEWHEEL:
				if (self && self->tab == 0)
				{
					POINT pt;
					GetCursorPos (&pt);
					ScreenToClient (h, &pt);
					const int tg = self->onGrace (pt.x, pt.y) ? 1 : 0;
					const double range = tg == 1 ? 50.0 : self->mainRange (); // grace moves in 10 ms steps
					const double step = (GET_WHEEL_DELTA_WPARAM (wp) / 120.0) / range;
					const ParamID pid = tg == 1 ? kVadGrace : self->mainParam ();
					self->ctrl->beginEdit (pid);
					self->setNorm (self->normFor (tg) + step, tg);
					self->ctrl->endEdit (pid);
				}
				return 0;
		}
		return DefWindowProcW (h, m, wp, lp);
	}

	EditController* ctrl;
	std::atomic<double>* dbValue;
	std::atomic<bool>* byp;
	std::atomic<int>* meth;
	const std::atomic<double>* rate;
	std::atomic<double>*vthr, *vgrace;
	const std::atomic<float>* vprob;
	Analyzer *aIn, *aOut;
	HWND hwnd = nullptr;
	std::thread ticker;
	std::atomic<bool> running {false}, pending {false};
	bool dragging = false, dirty = true, hoverSlider = false;
	int tab = 0, lastX = 0, lastSentDb = 0, skip = 0;
	bool editing = false, editFresh = false, menuOpen = false;
	int editTarget = 0;
	int hoverInfo = -1; // which "i" icon the mouse is over
	int hoverRow = -1, dragTarget = 0; // 0 = main slider, 1 = grace slider
	float vadShown = 0.f, rVad = 0.f;
	double lastThr = 1e9, lastGrace = 1e9;
	DWORD caretPhase = 0;
	std::wstring editBuf;
	DWORD caretT = 0;
	double dragNorm = 0.0, lastDb = 1e9;
	float dIn[Analyzer::kBands], dOut[Analyzer::kBands], sIn[Analyzer::kBands], sOut[Analyzer::kBands];
	float rIn[Analyzer::kBands], rOut[Analyzer::kBands], lvIn, lvOut, rLvIn = -120.f, rLvOut = -120.f;

	ComPtr<ID2D1Factory> d2d;
	ComPtr<IDWriteFactory> dw;
	ComPtr<IDWriteFactory5> dw5;
	ComPtr<IDWriteInMemoryFontFileLoader> memLoader;
	ComPtr<IDWriteFontCollection1> brandColl;
	std::wstring brandFamily;
	ComPtr<ID2D1HwndRenderTarget> rt;
	ComPtr<ID2D1BitmapRenderTarget> staticRT;
	ComPtr<ID2D1Bitmap> staticBmp;
	ID2D1RenderTarget* cur = nullptr;
	int staticTab = -1, staticMethod = -1;
	ComPtr<ID2D1SolidColorBrush> solid;
	ComPtr<ID2D1LinearGradientBrush> grad[kGrads];
	std::wstring fontFamily;
	std::map<int, ComPtr<IDWriteTextFormat>> formats;
	std::map<std::wstring, ComPtr<IDWriteTextLayout>> layouts;
};

//------------------------------------------------------------------------
class SpeexNoiseSuppression : public SingleComponentEffect
{
public:
	IPlugView* PLUGIN_API createView (FIDString name) SMTG_OVERRIDE
	{
		if (FIDStringsEqual (name, ViewType::kEditor))
			return new SpeexView (this, &suppressDb, &bypassed, &method, &sampleRate, &vadThr, &vadGrace, &vadProb, &inAnalyzer, &outAnalyzer);
		return nullptr;
	}

	static FUnknown* create (void*) { return static_cast<IAudioProcessor*> (new SpeexNoiseSuppression); }

	tresult PLUGIN_API initialize (FUnknown* ctx) SMTG_OVERRIDE
	{
		tresult r = SingleComponentEffect::initialize (ctx);
		if (r != kResultOk)
			return r;
		addAudioInput (STR16 ("Input"), SpeakerArr::kStereo);
		addAudioOutput (STR16 ("Output"), SpeakerArr::kStereo);
		auto* p = new RangeParameter (STR16 ("Suppression Level"), kSuppression, STR16 ("dB"), kMinDb, kMaxDb,
		                              kDefaultDb, 0, ParameterInfo::kCanAutomate);
		p->setPrecision (0);
		parameters.addParameter (p);
		parameters.addParameter (new Parameter (STR16 ("Bypass"), kBypass, nullptr, 0., 1,
		                                        ParameterInfo::kCanAutomate | ParameterInfo::kIsBypass));
		auto* m = new StringListParameter (STR16 ("Method"), kMethod);
		m->appendString (STR16 ("Speex"));
		m->appendString (STR16 ("RNNoise"));
		m->appendString (STR16 ("RNNoise Lite"));
		parameters.addParameter (m);
		auto* vt = new RangeParameter (STR16 ("VAD Threshold"), kVadThreshold, STR16 ("%"), 0., 100., 0., 0, ParameterInfo::kCanAutomate);
		vt->setPrecision (0);
		parameters.addParameter (vt);
		auto* vg = new RangeParameter (STR16 ("VAD Grace Period"), kVadGrace, STR16 ("ms"), 0., 500., 200., 0, ParameterInfo::kCanAutomate);
		vg->setPrecision (0);
		parameters.addParameter (vg);
		return kResultOk;
	}

	tresult PLUGIN_API terminate () SMTG_OVERRIDE
	{
		destroyStates ();
		return SingleComponentEffect::terminate ();
	}

	tresult PLUGIN_API setBusArrangements (SpeakerArrangement* in, int32 numIns, SpeakerArrangement* out,
	                                       int32 numOuts) SMTG_OVERRIDE
	{
		if (numIns != 1 || numOuts != 1 || in[0] != out[0])
			return kResultFalse;
		if (in[0] != SpeakerArr::kMono && in[0] != SpeakerArr::kStereo)
			return kResultFalse;
		removeAudioBusses ();
		addAudioInput (STR16 ("Input"), in[0]);
		addAudioOutput (STR16 ("Output"), out[0]);
		return kResultTrue;
	}

	tresult PLUGIN_API canProcessSampleSize (int32 s) SMTG_OVERRIDE
	{
		return s == kSample32 ? kResultTrue : kResultFalse;
	}

	uint32 PLUGIN_API getLatencySamples () SMTG_OVERRIDE { return static_cast<uint32> (frame); }

	tresult PLUGIN_API setupProcessing (ProcessSetup& setup) SMTG_OVERRIDE
	{
		sampleRate = setup.sampleRate;
		inAnalyzer.setRate (sampleRate);
		outAnalyzer.setRate (sampleRate);
		frame = std::max (1, static_cast<int> (std::lround (sampleRate / 100.0))); // 10 ms, like OBS
		destroyStates ();
		return SingleComponentEffect::setupProcessing (setup);
	}

	tresult PLUGIN_API setActive (TBool state) SMTG_OVERRIDE
	{
		if (state)
			createStates (2);
		else
			destroyStates ();
		return SingleComponentEffect::setActive (state);
	}

	tresult PLUGIN_API process (ProcessData& data) SMTG_OVERRIDE
	{
		if (data.inputParameterChanges)
		{
			int32 n = data.inputParameterChanges->getParameterCount ();
			for (int32 i = 0; i < n; ++i)
			{
				IParamValueQueue* q = data.inputParameterChanges->getParameterData (i);
				if (!q)
					continue;
				ParamValue v;
				int32 off;
				if (q->getParameterId () == kSuppression)
				{
					if (q->getPoint (q->getPointCount () - 1, off, v) == kResultTrue)
						setNormalized (v);
				}
				else if (q->getParameterId () == kBypass || q->getParameterId () == kMethod ||
				         q->getParameterId () == kVadThreshold || q->getParameterId () == kVadGrace)
				{
					if (q->getPoint (q->getPointCount () - 1, off, v) == kResultTrue)
						setParamNormalized (q->getParameterId (), v);
				}
			}
		}

		if (data.numInputs < 1 || data.numOutputs < 1 || data.numSamples <= 0)
			return kResultOk;

		AudioBusBuffers& in = data.inputs[0];
		AudioBusBuffers& out = data.outputs[0];
		int32 chans = std::min (in.numChannels, out.numChannels);
		if (states.empty () || chans > static_cast<int32> (states.size ()))
			createStates (chans);

		const float bypassTarget = bypassed.load () ? 1.f : 0.f;
		int level = static_cast<int> (std::lround (suppressDb.load ()));
		const int meth = method.load ();
		const bool useRn = meth >= 1 && frame == 480 && std::fabs (sampleRate.load ()- 48000.0) < 1.0;
		const float vadThrNow = static_cast<float> (vadThr.load ()) / 100.f;
		const int graceFrames = static_cast<int> (std::ceil (vadGrace.load () / 10.0));
		for (int32 c = 0; c < chans; ++c)
		{
			Chan& ch = states[c];
			if (ch.appliedLevel != level)
			{
				speex_preprocess_ctl (ch.st, SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, &level);
				ch.appliedLevel = level;
			}
			const float* src = in.channelBuffers32[c];
			float* dst = out.channelBuffers32[c];
			for (int32 i = 0; i < data.numSamples; ++i)
			{
				float x = std::clamp (src[i], -1.f, 1.f);
					ch.inQ[ch.fill] = static_cast<spx_int16_t> (std::lround (x * 32767.f));
				ch.inF[ch.fill] = x * 32768.f;
				float dry = ch.dry[ch.fill]; // input delayed by the same 10 ms, so bypass keeps the latency
				ch.dry[ch.fill] = x;
				float voice;
				ch.gate += (ch.gateTarget - ch.gate) * 0.02f; // voice gate, smoothed (~1 ms)
				if (useRn)
					voice = ch.outF[ch.fill] * (1.f / 32768.f) * ch.gate;
				else
					voice = ch.outQ[ch.fill] * (1.f / 32768.f);
				if (std::fabs (voice) < 4e-5f) // never output digital silence: some apps treat it as "mic dead"
				{
					ch.rng = ch.rng * 1664525u + 1013904223u;
					uint32_t r1 = ch.rng >> 8;
					ch.rng = ch.rng * 1664525u + 1013904223u;
					voice += (static_cast<float> (r1) - static_cast<float> (ch.rng >> 8)) * (1e-4f / 16777216.f);
				}
				ch.gb += (bypassTarget - ch.gb) * 0.01f; // short crossfade, no clicks
				dst[i] = voice + (dry - voice) * ch.gb;
				if (c == 0)
				{
					inAnalyzer.push (x);
					outAnalyzer.push (dst[i]);
				}
				if (++ch.fill == frame)
				{
					if (useRn)
					{
						float p = rnnoise_process_frame (meth == 2 ? ch.rnL : ch.rn, ch.outF.data (), ch.inF.data ());
						if (vadThrNow <= 0.001f)
						{
							ch.hold = 0;
							ch.gateTarget = 1.f;
						}
						else if (p >= vadThrNow)
						{
							ch.hold = graceFrames;
							ch.gateTarget = 1.f;
						}
						else if (ch.hold > 0)
						{
							--ch.hold;
							ch.gateTarget = 1.f;
						}
						else
							ch.gateTarget = 0.f;
						if (c == 0)
							vadProb.store (p, std::memory_order_relaxed);
					}
					else
					{
						speex_preprocess_run (ch.st, ch.inQ.data ());
						ch.outQ = ch.inQ;
					}
					ch.fill = 0;
				}
			}
		}
		return kResultOk;
	}

	tresult PLUGIN_API setState (IBStream* s) SMTG_OVERRIDE
	{
		IBStreamer r (s, kLittleEndian);
		double db;
		if (!r.readDouble (db))
			return kResultFalse;
		setNormalized ((std::clamp (db, kMinDb, kMaxDb) - kMinDb) / (kMaxDb - kMinDb));
		int32 b;
		if (r.readInt32 (b))
			setParamNormalized (kBypass, b ? 1.0 : 0.0);
		if (r.readInt32 (b))
			setParamNormalized (kMethod, std::clamp (b, 0, 2) / 2.0);
		double d;
		if (r.readDouble (d))
			setParamNormalized (kVadThreshold, std::clamp (d, 0.0, 100.0) / 100.0);
		if (r.readDouble (d))
			setParamNormalized (kVadGrace, std::clamp (d, 0.0, 500.0) / 500.0);
		return kResultOk;
	}

	tresult PLUGIN_API getState (IBStream* s) SMTG_OVERRIDE
	{
		IBStreamer w (s, kLittleEndian);
		w.writeDouble (suppressDb.load ());
		w.writeInt32 (bypassed.load () ? 1 : 0);
		w.writeInt32 (method.load ());
		w.writeDouble (vadThr.load ());
		w.writeDouble (vadGrace.load ());
		return kResultOk;
	}

	tresult PLUGIN_API setParamNormalized (ParamID id, ParamValue v) SMTG_OVERRIDE
	{
		if (id == kSuppression)
			suppressDb = kMinDb + v * (kMaxDb - kMinDb);
		else if (id == kBypass)
			bypassed = v > 0.5;
		else if (id == kMethod)
			method = std::clamp (static_cast<int> (std::lround (v * 2.0)), 0, 2);
		else if (id == kVadThreshold)
			vadThr = v * 100.0;
		else if (id == kVadGrace)
			vadGrace = v * 500.0;
		return SingleComponentEffect::setParamNormalized (id, v);
	}

private:
	struct Chan
	{
		SpeexPreprocessState* st = nullptr;
		DenoiseState *rn = nullptr, *rnL = nullptr;
		float gate = 1.f, gateTarget = 1.f;
		int hold = 0;
		std::vector<spx_int16_t> inQ, outQ;
		std::vector<float> inF, outF, dry;
		uint32_t rng = 12345u;
		float gb = 0.f;
		int fill = 0;
		int appliedLevel = 1;
	};

	void setNormalized (ParamValue v)
	{
		v = std::clamp (v, 0.0, 1.0);
		suppressDb = kMinDb + v * (kMaxDb - kMinDb);
		SingleComponentEffect::setParamNormalized (kSuppression, v);
	}

	void createStates (int chans)
	{
		destroyStates ();
		states.resize (chans);
		for (auto& ch : states)
		{
			ch.st = speex_preprocess_state_init (frame, static_cast<spx_int32_t> (sampleRate));
			spx_int32_t on = 1, off = 0; // VAD is off by default
			speex_preprocess_ctl (ch.st, SPEEX_PREPROCESS_SET_DENOISE, &on);
			speex_preprocess_ctl (ch.st, SPEEX_PREPROCESS_SET_AGC, &off);
			speex_preprocess_ctl (ch.st, SPEEX_PREPROCESS_SET_DEREVERB, &off);
			ch.rn = rnnoise_create (nullptr);
			ch.rnL = elnoise_create_light ();
			ch.rng = 12345u + static_cast<uint32_t> (&ch - states.data ()) * 7919u;
			ch.inQ.assign (frame, 0);
			ch.outQ.assign (frame, 0);
			ch.inF.assign (frame, 0.f);
			ch.outF.assign (frame, 0.f);
			ch.dry.assign (frame, 0.f);
			ch.gb = bypassed.load () ? 1.f : 0.f;
		}
	}

	void destroyStates ()
	{
		for (auto& ch : states)
		{
			if (ch.st)
				speex_preprocess_state_destroy (ch.st);
			if (ch.rn)
				rnnoise_destroy (ch.rn);
			if (ch.rnL)
				rnnoise_destroy (ch.rnL);
		}
		states.clear ();
	}

	Analyzer inAnalyzer, outAnalyzer;
	std::vector<Chan> states;
	std::atomic<double> suppressDb {kDefaultDb};
	std::atomic<bool> bypassed {false};
	std::atomic<int> method {0};
	std::atomic<double> vadThr {0.0}, vadGrace {200.0};
	std::atomic<float> vadProb {0.f};
	std::atomic<double> sampleRate {48000.0};
	int frame = 480;
};

BEGIN_FACTORY_DEF ("elem3n7ary", "https://github.com/elem3n7ary/elnoise", "")
	DEF_CLASS2 (INLINE_UID_FROM_FUID (kPluginUID), PClassInfo::kManyInstances, kVstAudioEffectClass,
	            "elNoise", Vst::kSimpleModeSupported, Vst::PlugType::kFxRestoration, "1.2.0",
	            kVstVersionString, SpeexNoiseSuppression::create)
END_FACTORY


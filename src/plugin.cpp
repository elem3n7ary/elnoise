// SPDX-License-Identifier: GPL-3.0-or-later
// Speex Noise Suppression - VST3 plugin
// Copyright (C) 2026 elem3n7ary
//
// This program is free software: you can redistribute it and/or modify it under the terms of the
// GNU General Public License as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version. It is distributed WITHOUT ANY WARRANTY.
// See the LICENSE file. Bundled third-party code: see THIRD_PARTY_LICENSES/.

// Speex noise suppression VST3 - mirrors OBS "Noise Suppression" filter (Speex method).
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
#include <dwrite_1.h>
#include <dwmapi.h>
#include <wrl/client.h>
#pragma comment(lib, "d2d1.lib")
#pragma comment(lib, "dwrite.lib")
#pragma comment(lib, "dwmapi.lib")

#include <speex/speex_preprocess.h>

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

enum : ParamID { kSuppression = 100 };
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
	SpeexView (EditController* c, std::atomic<double>* db, Analyzer* in, Analyzer* out)
	: CPluginView (nullptr), ctrl (c), dbValue (db), aIn (in), aOut (out)
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
	static constexpr float kTrackL = 44.f, kTrackR = kW - 44.f, kTrackY = 372.f;
	static constexpr float kTabX0 = kW - 178.f, kTabMid = kW - 99.f, kTabX1 = kW - 20.f;

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
	double norm () const { return (dbValue->load () - kMinDb) / (kMaxDb - kMinDb); }
	double curNorm () const { return dragging ? dragNorm : norm (); }

	// While dragging, the view tracks the value locally and only talks to the host when the
	// rounded dB value changes (Speex only takes whole dB anyway).
	void setNorm (double t)
	{
		t = std::clamp (t, 0.0, 1.0);
		dirty = true;
		if (dragging)
		{
			dragNorm = t;
			int db = static_cast<int> (std::lround (kMinDb + t * (kMaxDb - kMinDb)));
			if (db != lastSentDb)
			{
				lastSentDb = db;
				ctrl->setParamNormalized (kSuppression, t);
				ctrl->performEdit (kSuppression, t);
			}
			return;
		}
		ctrl->setParamNormalized (kSuppression, t);
		ctrl->performEdit (kSuppression, t);
	}

	bool onSlider (int x, int y) const
	{
		return y >= 330 && y <= 418 && x >= kTrackL - 20 && x <= kTrackR + 20;
	}
	static constexpr float kLcdX = kW - 156.f, kLcdY = 292.f, kLcdW = 128.f, kLcdH = 46.f;
	bool hitLcd (int x, int y) const
	{
		return x >= kLcdX && x <= kLcdX + kLcdW && y >= kLcdY && y <= kLcdY + kLcdH;
	}

	// --- inline numeric entry in the LCD box
	void startEdit ()
	{
		editing = true;
		editFresh = true; // first typed character replaces the whole value
		editBuf = std::to_wstring (static_cast<int> (std::lround (kMinDb + curNorm () * (kMaxDb - kMinDb))));
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
		if (v > 0)
			v = -v; // the range is 0..-60, so "30" means -30
		v = std::clamp<long> (v, static_cast<long> (kMinDb), static_cast<long> (kMaxDb));
		ctrl->beginEdit (kSuppression);
		setNorm ((v - kMinDb) / (kMaxDb - kMinDb));
		ctrl->endEdit (kSuppression);
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
		else if ((ch >= L'0' && ch <= L'9') || ch == L'-')
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

	IDWriteTextLayout* layoutFor (const wchar_t* s, float size, DWRITE_FONT_WEIGHT w, Al al, float track)
	{
		std::wstring key = s;
		key += L'\x1';
		key += std::to_wstring (static_cast<int> (size * 10)) + L'/' + std::to_wstring (w) + L'/' +
		       std::to_wstring (al) + L'/' + std::to_wstring (static_cast<int> (track * 10));
		auto it = layouts.find (key);
		if (it != layouts.end ())
			return it->second.Get ();
		if (layouts.size () > 500)
			layouts.clear ();
		int fk = static_cast<int> (size * 10) * 1000 + w + al;
		auto& fmt = formats[fk];
		if (!fmt)
		{
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
	           float track = 0.f)
	{
		IDWriteTextLayout* l = layoutFor (s, size, w, al, track);
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

		// logo
		strokeRR (20, 15, 28, 28, 7, C (205, 205, 210), 1.8f);
		text (L"el", 17, DWRITE_FONT_WEIGHT_SEMI_BOLD, C (220, 220, 225), 34, 16.5f, kCenter);

		text (L"SPEEX NOISE SUPPRESSION", 15, DWRITE_FONT_WEIGHT_SEMI_BOLD, C (208, 208, 213), 62, 20, kLeft, 2.2f);

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

	void drawSlider ()
	{
		line (0, 283.5f, kW, 283.5f, C (0, 0, 0, 160));
		line (0, 284.5f, kW, 284.5f, C (255, 255, 255, 20));

		text (L"SUPPRESSION", 13, DWRITE_FONT_WEIGHT_SEMI_BOLD, cText, 28, 298, kLeft, 2.2f);
		text (L"Lower value = stronger noise reduction", 11, DWRITE_FONT_WEIGHT_NORMAL, cMuted, 28, 320, kLeft);

		// LCD
		const float bx = kW - 156.f, by = 292.f, bw = 128.f, bh = 46.f;
		cur->FillRoundedRectangle (RR (bx, by, bw, bh, 8), vgrad (gLcd, by, by + bh));
		strokeRR (bx + 0.5f, by + 0.5f, bw - 1, bh - 1, 8, C (0, 0, 0), 1.f);
		line (bx + 6, by + bh + 1.5f, bx + bw - 6, by + bh + 1.5f, C (255, 255, 255, 24));
		line (bx + 8, by + 1.5f, bx + bw - 8, by + 1.5f, C (255, 255, 255, 12));
		text (L"dB", 13, DWRITE_FONT_WEIGHT_NORMAL, cMuted, bx + bw - 14, by + 21, kRight);

		// groove
		fillRR (kTrackL - 5, kTrackY - 5, kTrackR - kTrackL + 10, 10, 5, C (6, 6, 7));
		strokeRR (kTrackL - 4.5f, kTrackY - 4.5f, kTrackR - kTrackL + 9, 9, 4.5f, C (66, 66, 72), 1.f);
		line (kTrackL, kTrackY - 3.5f, kTrackR, kTrackY - 3.5f, C (0, 0, 0, 160));
		for (int i = 0; i <= 30; i += 5)
		{
			float x = std::floor (kTrackL + (kTrackR - kTrackL) * i / 30.f) + 0.5f;
			wchar_t lb[8];
			swprintf (lb, 8, L"%d", -60 + i * 2);
			text (lb, 10, DWRITE_FONT_WEIGHT_NORMAL, C (140, 140, 146), x, kTrackY + 28.f, kCenter);
		}
	}

	void drawSliderDynamic ()
	{
		const float bx = kLcdX, by = kLcdY, bw = kLcdW;
		double t = curNorm ();
		wchar_t b[16];
		swprintf (b, 16, L"%d", static_cast<int> (std::lround (kMinDb + t * (kMaxDb - kMinDb))));
		if (!editing)
			text (b, 30, DWRITE_FONT_WEIGHT_LIGHT, cWhite, bx + bw - 44, by + 4.5f, kRight);
		else
		{
			strokeRR (bx + 0.5f, by + 0.5f, bw - 1, kLcdH - 1, 8, C (255, 255, 255, 150), 1.5f);
			const float right = bx + bw - 44;
			float w = 0.f;
			if (!editBuf.empty ())
			{
				IDWriteTextLayout* l = layoutFor (editBuf.c_str (), 30, DWRITE_FONT_WEIGHT_LIGHT, kRight, 0.f);
				DWRITE_TEXT_METRICS m {};
				l->GetMetrics (&m);
				w = m.widthIncludingTrailingWhitespace;
			}
			if (editFresh && w > 0.f)
				fillRect (right - w - 3, by + 7, w + 6, kLcdH - 14, C (255, 255, 255, 60));
			if (!editBuf.empty ())
				text (editBuf.c_str (), 30, DWRITE_FONT_WEIGHT_LIGHT, cWhite, right, by + 4.5f, kRight);
			if (((GetTickCount () - caretT) / 530) % 2 == 0 && !editFresh)
				line (right + 2, by + 9, right + 2, by + kLcdH - 9, cWhite, 1.5f);
		}

		float kx = static_cast<float> (kTrackL + t * (kTrackR - kTrackL));
		if (kx > kTrackL + 1)
		{
			fillRR (kTrackL - 3, kTrackY - 3, kx - kTrackL + 3, 6, 3, C (255, 255, 255, 34));
			cur->FillRoundedRectangle (RR (kTrackL - 3, kTrackY - 2.5f, kx - kTrackL + 3, 5, 2.5f), vgrad (gSlider, kTrackY - 3, kTrackY + 3));
		}
		// ticks + labels
		for (int i = 0; i <= 30; ++i)
		{
			float x = std::floor (kTrackL + (kTrackR - kTrackL) * i / 30.f) + 0.5f;
			bool major = i % 5 == 0, lit = x <= kx + 0.5f;
			line (x, kTrackY + 16.f, x, kTrackY + (major ? 26.f : 21.f), lit ? C (220, 220, 225) : cDim);
		}
		// knob
		for (int i = 4; i >= 1; --i)
			fillRR (kx - 11 - i, kTrackY - 19 + i + 3, 22 + 2 * i, 38, 9 + i, C (0, 0, 0, 22));
		Grad kg = (dragging || hoverSlider) ? gKnobHot : gKnob;
		cur->FillRoundedRectangle (RR (kx - 11, kTrackY - 20, 22, 40, 9), vgrad (kg, kTrackY - 20, kTrackY + 20));
		strokeRR (kx - 10.5f, kTrackY - 19.5f, 21, 39, 8.5f, C (14, 14, 16), 1.f);
		line (kx - 6, kTrackY - 18, kx + 6, kTrackY - 18, C (255, 255, 255, 150));
		for (float gx : {-4.f, 0.f, 4.f})
			line (kx + gx, kTrackY - 9, kx + gx, kTrackY + 9, C (58, 58, 64), 1.5f);
	}

	void drawAbout ()
	{
		strokeRR (kW / 2.f - 34, 96, 68, 68, 16, C (205, 205, 210), 3.f);
		text (L"el", 38, DWRITE_FONT_WEIGHT_SEMI_BOLD, C (225, 225, 230), kW / 2.f, 107, kCenter);
		text (L"Speex Noise Suppression", 24, DWRITE_FONT_WEIGHT_SEMI_BOLD, cWhite, kW / 2.f, 186, kCenter, 0.6f);
		text (L"VST3  |  version 1.0.0", 13, DWRITE_FONT_WEIGHT_NORMAL, cMuted, kW / 2.f, 222, kCenter, 0.4f);
		line (kW / 2.f - 60, 262.5f, kW / 2.f + 60, 262.5f, C (255, 255, 255, 40));
		text (L"MADE BY", 11, DWRITE_FONT_WEIGHT_SEMI_BOLD, cMuted, kW / 2.f, 282, kCenter, 2.5f);
		text (L"elem3n7ary", 32, DWRITE_FONT_WEIGHT_SEMI_BOLD, cWhite, kW / 2.f, 300, kCenter, 0.5f);
		text (L"Based on SpeexDSP (Xiph.Org, BSD license)", 12, DWRITE_FONT_WEIGHT_NORMAL, cMuted, kW / 2.f, 380, kCenter);
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
	}

	void render ()
	{
		if (!ensureTarget ())
			return;
		if (staticTab != tab || !staticBmp)
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
		if (rt->EndDraw () == D2DERR_RECREATE_TARGET)
		{
			releaseTarget ();
			dirty = true;
		}
		std::copy (sIn, sIn + Analyzer::kBands, rIn);
		std::copy (sOut, sOut + Analyzer::kBands, rOut);
		rLvIn = lvIn;
		rLvOut = lvOut;
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

		float diff = std::fabs (lvIn - rLvIn) + std::fabs (lvOut - rLvOut);
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
		double cur = dbValue->load ();
		if (std::fabs (cur - lastDb) > 1e-6)
		{
			lastDb = cur;
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
				if (self->editing && !self->hitLcd (x, y))
					self->commitEdit ();
				if (self->tab == 0 && self->hitLcd (x, y))
				{
					if (!self->editing)
						self->startEdit ();
					return 0;
				}
				int t = self->hitTab (x, y);
				if (t >= 0)
				{
					self->tab = t;
					self->dirty = true;
				}
				else if (self->tab == 0 && self->onSlider (x, y))
				{
					self->dragging = true;
					self->dragNorm = self->norm ();
					self->lastSentDb = static_cast<int> (std::lround (self->dbValue->load ()));
					self->lastX = x;
					SetCapture (h);
					self->ctrl->beginEdit (kSuppression);
					if (!(GetKeyState (VK_SHIFT) & 0x8000))
						self->setNorm ((x - kTrackL) / double (kTrackR - kTrackL));
				}
				return 0;
			}
			case WM_MOUSEMOVE:
				if (!self)
					return 0;
				if (self->dragging)
				{
					if (GetKeyState (VK_SHIFT) & 0x8000) // fine adjust: 5x slower, relative
						self->setNorm (self->curNorm () + (x - self->lastX) / (5.0 * (kTrackR - kTrackL)));
					else
						self->setNorm ((x - kTrackL) / double (kTrackR - kTrackL));
					self->lastX = x;
				}
				else
				{
					bool hv = self->tab == 0 && self->onSlider (x, y);
					if (hv != self->hoverSlider)
					{
						self->hoverSlider = hv;
						self->dirty = true;
					}
					TRACKMOUSEEVENT te {sizeof (te), TME_LEAVE, h, 0};
					TrackMouseEvent (&te);
				}
				return 0;
			case WM_MOUSELEAVE:
				if (self && self->hoverSlider)
				{
					self->hoverSlider = false;
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
				bool hand = self && (self->hitTab (pt.x, pt.y) >= 0 || (self->tab == 0 && (self->dragging || self->onSlider (pt.x, pt.y))));
				bool ibeam = self && self->tab == 0 && self->hitLcd (pt.x, pt.y);
				SetCursor (LoadCursor (nullptr, ibeam ? IDC_IBEAM : (hand ? IDC_HAND : IDC_ARROW)));
				return TRUE;
			}
			case WM_LBUTTONUP:
				if (self && self->dragging)
				{
					self->dragging = false;
					ReleaseCapture ();
					self->ctrl->setParamNormalized (kSuppression, self->dragNorm);
					self->ctrl->performEdit (kSuppression, self->dragNorm);
					self->ctrl->endEdit (kSuppression);
					self->dirty = true;
				}
				return 0;
			case WM_MOUSEWHEEL:
				if (self && self->tab == 0)
				{
					double step = (GET_WHEEL_DELTA_WPARAM (wp) / 120.0) / (kMaxDb - kMinDb);
					self->ctrl->beginEdit (kSuppression);
					self->setNorm (self->norm () + step);
					self->ctrl->endEdit (kSuppression);
				}
				return 0;
		}
		return DefWindowProcW (h, m, wp, lp);
	}

	EditController* ctrl;
	std::atomic<double>* dbValue;
	Analyzer *aIn, *aOut;
	HWND hwnd = nullptr;
	std::thread ticker;
	std::atomic<bool> running {false}, pending {false};
	bool dragging = false, dirty = true, hoverSlider = false;
	int tab = 0, lastX = 0, lastSentDb = 0, skip = 0;
	bool editing = false, editFresh = false;
	DWORD caretPhase = 0;
	std::wstring editBuf;
	DWORD caretT = 0;
	double dragNorm = 0.0, lastDb = 1e9;
	float dIn[Analyzer::kBands], dOut[Analyzer::kBands], sIn[Analyzer::kBands], sOut[Analyzer::kBands];
	float rIn[Analyzer::kBands], rOut[Analyzer::kBands], lvIn, lvOut, rLvIn = -120.f, rLvOut = -120.f;

	ComPtr<ID2D1Factory> d2d;
	ComPtr<IDWriteFactory> dw;
	ComPtr<ID2D1HwndRenderTarget> rt;
	ComPtr<ID2D1BitmapRenderTarget> staticRT;
	ComPtr<ID2D1Bitmap> staticBmp;
	ID2D1RenderTarget* cur = nullptr;
	int staticTab = -1;
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
			return new SpeexView (this, &suppressDb, &inAnalyzer, &outAnalyzer);
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
				if (q && q->getParameterId () == kSuppression)
				{
					ParamValue v;
					int32 off;
					if (q->getPoint (q->getPointCount () - 1, off, v) == kResultTrue)
						setNormalized (v);
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

		int level = static_cast<int> (std::lround (suppressDb.load ()));
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
				dst[i] = ch.outQ[ch.fill] * (1.f / 32768.f);
				if (c == 0)
				{
					inAnalyzer.push (x);
					outAnalyzer.push (dst[i]);
				}
				if (++ch.fill == frame)
				{
					speex_preprocess_run (ch.st, ch.inQ.data ());
					ch.outQ = ch.inQ;
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
		return kResultOk;
	}

	tresult PLUGIN_API getState (IBStream* s) SMTG_OVERRIDE
	{
		IBStreamer w (s, kLittleEndian);
		w.writeDouble (suppressDb.load ());
		return kResultOk;
	}

	tresult PLUGIN_API setParamNormalized (ParamID id, ParamValue v) SMTG_OVERRIDE
	{
		if (id == kSuppression)
			suppressDb = kMinDb + v * (kMaxDb - kMinDb);
		return SingleComponentEffect::setParamNormalized (id, v);
	}

private:
	struct Chan
	{
		SpeexPreprocessState* st = nullptr;
		std::vector<spx_int16_t> inQ, outQ;
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
			spx_int32_t on = 1, off = 0;
			speex_preprocess_ctl (ch.st, SPEEX_PREPROCESS_SET_DENOISE, &on);
			speex_preprocess_ctl (ch.st, SPEEX_PREPROCESS_SET_AGC, &off);
			speex_preprocess_ctl (ch.st, SPEEX_PREPROCESS_SET_VAD, &off);
			speex_preprocess_ctl (ch.st, SPEEX_PREPROCESS_SET_DEREVERB, &off);
			ch.inQ.assign (frame, 0);
			ch.outQ.assign (frame, 0);
		}
	}

	void destroyStates ()
	{
		for (auto& ch : states)
			if (ch.st)
				speex_preprocess_state_destroy (ch.st);
		states.clear ();
	}

	Analyzer inAnalyzer, outAnalyzer;
	std::vector<Chan> states;
	std::atomic<double> suppressDb {kDefaultDb};
	double sampleRate = 48000.0;
	int frame = 480;
};

BEGIN_FACTORY_DEF ("Speex NS", "", "")
	DEF_CLASS2 (INLINE_UID_FROM_FUID (kPluginUID), PClassInfo::kManyInstances, kVstAudioEffectClass,
	            "Speex Noise Suppression", Vst::kSimpleModeSupported, Vst::PlugType::kFxRestoration, "1.0.0",
	            kVstVersionString, SpeexNoiseSuppression::create)
END_FACTORY


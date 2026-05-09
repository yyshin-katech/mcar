// qt_hmi/style/F1Tokens.h
//
// F1 telemetry HMI design tokens — direct port of `web_hmi/web/f1/F1HMI.jsx`
// const T palette. Palette values must stay byte-for-byte identical to the
// JSX source so a side-by-side comparison with the web HMI shows no colour
// drift.
//
// Use freely from any widget paintEvent. All members are constexpr-light
// inline statics (QColor cannot be constexpr in Qt5, so namespace-scope
// `const` instead).
//
// See claude_work_list/hmi_design/01_design.md §5/M2.
#pragma once

#include <QtGui/QColor>
#include <QtCore/QString>

namespace f1tokens {

// Background ladder
inline const QColor bg0   {  6,   9,  12};   // "#06090c" — body background
inline const QColor bg1   { 10,  14,  19};   // "#0a0e13" — section header strip
inline const QColor bg2   { 15,  20,  25};   // "#0f1419"
inline const QColor bg3   { 22,  28,  36};   // "#161c24"

// Hairlines / dividers
inline const QColor line   { 31,  39,  48};   // "#1f2730"
inline const QColor lineHi { 42,  52,  65};   // "#2a3441"

// Text ramp
inline const QColor text0 {230, 237, 243};   // "#e6edf3" — primary
inline const QColor text1 {192, 200, 210};   // "#c0c8d2"
inline const QColor text2 {122, 132, 146};   // "#7a8492"
inline const QColor text3 { 74,  84,  98};   // "#4a5462" — disabled / dim

// Accents
inline const QColor cyan    {  0, 229, 255};   // "#00e5ff"
inline const QColor cyanDim {  0, 149, 168};   // "#0095a8"
inline const QColor amber   {255, 181,  71};   // "#ffb547"
inline const QColor red     {255,  59,  59};   // "#ff3b3b"
inline const QColor green   { 34, 224, 154};   // "#22e09a"
inline const QColor magenta {255,  94, 168};   // "#ff5ea8"

// Helper to dim/saturate a token color with explicit alpha (0..255).
inline QColor with_alpha(const QColor& c, int a) {
  QColor r = c;
  r.setAlpha(a);
  return r;
}

// Monospace font family preference (JetBrains Mono if installed, else system
// monospace fallback per design §6.D — license unresolved so we don't ship
// the .ttf inside qt_hmi).
inline QString monoFamily() {
  return QStringLiteral("JetBrains Mono, DejaVu Sans Mono, Liberation Mono, "
                         "Monospace");
}

}  // namespace f1tokens

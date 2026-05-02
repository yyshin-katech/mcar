/* global React, ReactDOM, HMIScreen, DesignCanvas, DCSection, DCArtboard */

const Wrap = ({ children, scale = 0.55 }) => (
  <div style={{
    width: 1600 * scale,
    height: 900 * scale,
    overflow: "hidden",
    background: "#08090B",
    borderRadius: 8,
  }}>
    <div style={{
      width: 1600,
      height: 900,
      transform: `scale(${scale})`,
      transformOrigin: "top left",
    }}>
      {children}
    </div>
  </div>
);

const App = () => (
  <DesignCanvas title="Ioniq5 HMI · Variations">
    <DCSection id="vehicle-style" title="A · Vehicle View Style" subtitle="Top-down — solid model vs HUD wireframe">
      <DCArtboard id="v-solid" label="Solid (top-down)" width={880} height={495}>
        <Wrap><HMIScreen topView vehicleStyle="solid" accent="amber" /></Wrap>
      </DCArtboard>
      <DCArtboard id="v-wire" label="Wireframe / HUD (top-down)" width={880} height={495}>
        <Wrap><HMIScreen topView vehicleStyle="wireframe" accent="amber" /></Wrap>
      </DCArtboard>
    </DCSection>

    <DCSection id="accent" title="B · Accent Color" subtitle="Same charcoal base, different signal color (top-down)">
      <DCArtboard id="a-amber" label="Amber (default)" width={880} height={495}>
        <Wrap><HMIScreen topView accent="amber" /></Wrap>
      </DCArtboard>
      <DCArtboard id="a-cyan" label="Cyan teal" width={880} height={495}>
        <Wrap><HMIScreen topView accent="cyan" /></Wrap>
      </DCArtboard>
      <DCArtboard id="a-violet" label="Soft violet" width={880} height={495}>
        <Wrap><HMIScreen topView accent="violet" /></Wrap>
      </DCArtboard>
      <DCArtboard id="a-lime" label="Lime" width={880} height={495}>
        <Wrap><HMIScreen topView accent="lime" /></Wrap>
      </DCArtboard>
    </DCSection>

    <DCSection id="type" title="C · Typography" subtitle="Sans-serif (Inter) vs monospace (JetBrains) — top-down">
      <DCArtboard id="t-sans" label="Inter — sans" width={880} height={495}>
        <Wrap><HMIScreen topView typeface="sans" /></Wrap>
      </DCArtboard>
      <DCArtboard id="t-mono" label="JetBrains — mono" width={880} height={495}>
        <Wrap><HMIScreen topView typeface="mono" /></Wrap>
      </DCArtboard>
    </DCSection>
  </DesignCanvas>
);

ReactDOM.createRoot(document.getElementById("root")).render(<App />);

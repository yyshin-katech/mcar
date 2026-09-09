/* global React, window, LAYER_STYLE */

// Right-edge control panel — toggles for display options, camera mode,
// and HD map layer visibility. Pure DOM (no Three.js); state is owned
// by ThreeJSScreen and threaded down as props.

const LAYER_LABELS = {
  A1_NODE:                    'A1 Nodes',
  A2_LINK:                    'A2 Lane lines',
  A3_DRIVEWAYSECTION:         'A3 Driveway',
  A4_SUBSIDIARYSECTION:       'A4 Subsidiary',
  A5_PARKINGLOT:              'A5 Parking',
  B1_SAFETYSIGN:              'B1 Safety signs',
  B2_SURFACELINEMARK:         'B2 Surface lines',
  B3_SURFACEMARK:             'B3 Surface marks',
  C1_TRAFFICLIGHT:            'C1 Traffic light',
  C3_VEHICLEPROTECTIONSAFETY: 'C3 Protection',
  C4_SPEEDBUMP:               'C4 Speed bumps',
  C5_HEIGHTBARRIER:           'C5 Height barrier',
  C6_POSTPOINT:               'C6 Post points',
};

function Swatch({ color }) {
  const style = {
    display: 'inline-block', width: 9, height: 9,
    background: color, marginRight: 5, verticalAlign: 'middle',
    borderRadius: 1,
  };
  return <span style={style} />;
}

function Toggle({ checked, onChange, label }) {
  return (
    <label>
      <input type="checkbox" checked={!!checked}
             onChange={(e) => onChange(e.target.checked)} />
      {label}
    </label>
  );
}

function ControlPanel({
  layerVis, onLayerVis,
  showBoxes, onShowBoxes,
  showHeading, onShowHeading,
  showClouds, onShowClouds,
  pointSize, onPointSize,
  nearestOnly, onNearestOnly, nearestN,
  showSdsm, onShowSdsm,
  cameraMode, onCameraMode,
}) {
  const layerStyles = window.LAYER_STYLE || {};
  const layerNames = Object.keys(layerStyles);

  return (
    <div id="panel">
      <h3>Display</h3>
      <Toggle label="Bounding boxes" checked={showBoxes} onChange={onShowBoxes} />
      <Toggle label="Heading arrows" checked={showHeading} onChange={onShowHeading} />
      <Toggle label="Point clouds" checked={showClouds} onChange={onShowClouds} />
      {onNearestOnly && (
        <Toggle label={`Nearest ${nearestN || 5} only`}
                checked={nearestOnly} onChange={onNearestOnly} />
      )}
      {onShowSdsm && (
        <Toggle label="V2X SDSM objects"
                checked={showSdsm} onChange={onShowSdsm} />
      )}
      <label>
        Point size&nbsp;
        <input type="range" min="0.02" max="0.5" step="0.02"
               value={pointSize}
               style={{ width: 100, verticalAlign: 'middle' }}
               onChange={(e) => onPointSize(parseFloat(e.target.value))} />
        &nbsp;<span style={{ color: '#7a8492' }}>{pointSize.toFixed(2)}</span>
      </label>

      <h3>Camera</h3>
      <label>
        <input type="radio" name="cammode" value="iso"
               checked={cameraMode === 'iso'}
               onChange={() => onCameraMode('iso')} />
        Iso (chase)
      </label>
      <label>
        <input type="radio" name="cammode" value="top"
               checked={cameraMode === 'top'}
               onChange={() => onCameraMode('top')} />
        Top (bird's-eye)
      </label>

      <h3>Map layers</h3>
      {layerNames.map((name) => {
        const style = layerStyles[name] || { color: '#888' };
        const label = LAYER_LABELS[name] || name;
        return (
          <label key={name}>
            <input type="checkbox" checked={layerVis[name] !== false}
                   onChange={(e) => onLayerVis(name, e.target.checked)} />
            <Swatch color={style.color} />{label}
          </label>
        );
      })}
    </div>
  );
}

window.ControlPanel = ControlPanel;

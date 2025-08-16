import React, { useState } from 'react';
import './CollapsiblePanel.css';

// Simple collapsible panel for HUD sections
const CollapsiblePanel = ({ title, children }) => {
  const [open, setOpen] = useState(false);
  return (
    <div className="collapsible-panel">
      <div
        className="collapsible-header"
        onClick={() => setOpen((o) => !o)}
        style={{ cursor: 'pointer', fontWeight: 'bold', userSelect: 'none', padding: '6px 0' }}
      >
        {title} <span style={{ float: 'right' }}>{open ? '▲' : '▼'}</span>
      </div>
      {open && <div className="collapsible-content">{children}</div>}
    </div>
  );
};

export default CollapsiblePanel;

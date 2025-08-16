
import React, { useState } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
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
      <AnimatePresence initial={false}>
        {open && (
          <motion.div
            className="collapsible-content"
            initial={{ opacity: 0, y: -10 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: -10 }}
            transition={{ duration: 0.25 }}
          >
            {children}
          </motion.div>
        )}
      </AnimatePresence>
    </div>
  );
};

export default CollapsiblePanel;

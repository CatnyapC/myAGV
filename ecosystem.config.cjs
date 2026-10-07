const path = require('node:path');

module.exports = {
  apps: [
    {
      name: 'myagv-api',
      cwd: __dirname,
      script: path.join(__dirname, '.venv/bin/python'),
      interpreter: 'none',
      args: '-m web_backend.server --port 8791',
      env: { PYTHONUNBUFFERED: '1' },
      autorestart: true,
      exp_backoff_restart_delay: 100,
      max_memory_restart: '256M',
    },
    {
      name: 'myagv-web',
      cwd: path.join(__dirname, 'web'),
      script: path.join(__dirname, 'web/node_modules/vite/bin/vite.js'),
      interpreter: process.execPath,
      args: '--host 0.0.0.0 --port 5173 --strictPort',
      autorestart: true,
      exp_backoff_restart_delay: 100,
      max_memory_restart: '384M',
    },
  ],
};

const path = require('node:path');
const os = require('node:os');

module.exports = { apps: [
  {
    name: 'myagv-codex', cwd: __dirname,
    script: path.join(__dirname, '.venv/bin/python'), interpreter: 'none',
    args: '-m web_backend.codex_bridge',
    env: {
      PYTHONUNBUFFERED: '1',
      PATH: `${path.join(os.homedir(), '.local/bin')}:${process.env.PATH}`,
      MYAGV_CODEX_TOKEN_FILE: path.join(os.homedir(), '.config/myagv/codex-token'),
    },
    autorestart: true, exp_backoff_restart_delay: 1000,
  },
  {
    name: 'myagv-codex-tunnel', cwd: __dirname,
    script: '/usr/bin/ssh', interpreter: 'none',
    args: ['-N', '-T', '-o', 'BatchMode=yes', '-o', 'IdentitiesOnly=yes',
      '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=15',
      '-o', 'ServerAliveCountMax=3', '-o', 'ConnectTimeout=10',
      '-i', path.join(os.homedir(), '.ssh/myagv-codex-tunnel'),
      '-R', '127.0.0.1:9008:127.0.0.1:9008', 'er@172.28.13.26'],
    autorestart: true, exp_backoff_restart_delay: 1000,
  },
] };

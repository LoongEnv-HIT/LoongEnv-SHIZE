import { spawn } from 'child_process';

const dev = spawn('npm', ['run', 'dev'], {
  shell: true,
  stdio: 'inherit',
});

const api = spawn('npm', ['run', 'dev:api'], {
  shell: true,
  stdio: 'inherit',
});

let shuttingDown = false;

function shutdown(code = 0) {
  if (shuttingDown) return;
  shuttingDown = true;
  dev.kill();
  api.kill();
  process.exit(code);
}

process.on('SIGINT', () => shutdown(0));
process.on('SIGTERM', () => shutdown(0));

dev.on('exit', (code) => {
  if (!shuttingDown) {
    console.error(`[dev:stack] frontend exited with code ${code ?? 0}`);
    shutdown(code ?? 1);
  }
});

api.on('exit', (code) => {
  if (!shuttingDown) {
    console.error(`[dev:stack] sidecar exited with code ${code ?? 0}`);
    shutdown(code ?? 1);
  }
});

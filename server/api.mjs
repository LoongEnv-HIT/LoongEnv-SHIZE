import express from 'express';
import path from 'path';
import { fileURLToPath } from 'url';
import { spawn } from 'child_process';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const projectRoot = path.resolve(__dirname, '..');
const bridgeScript = path.resolve(projectRoot, 'scripts', 'planning_bridge.py');
const pythonBin = process.env.PLANNER_PYTHON || 'python';
const port = Number(process.env.PLANNER_API_PORT || 3001);
const apiVersion = '2026-04-02-sidecar-v2';

function runBridge(payload, timeoutMs = 300000) {
  return new Promise((resolve, reject) => {
    const child = spawn(pythonBin, [bridgeScript], {
      cwd: projectRoot,
      env: process.env,
      stdio: ['pipe', 'pipe', 'pipe'],
    });

    let stdout = '';
    let stderr = '';
    let settled = false;

    const timer = setTimeout(() => {
      if (settled) return;
      settled = true;
      child.kill('SIGKILL');
      reject(new Error(`Planner bridge timed out after ${timeoutMs}ms`));
    }, timeoutMs);

    child.stdout.on('data', (chunk) => {
      stdout += chunk.toString();
    });

    child.stderr.on('data', (chunk) => {
      stderr += chunk.toString();
    });

    child.on('error', (error) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      reject(error);
    });

    child.on('close', (code) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);

      const trimmed = stdout.trim();
      let parsed = null;
      if (trimmed.length > 0) {
        try {
          parsed = JSON.parse(trimmed);
        } catch {
          return reject(
            new Error(
              `Planner bridge returned non-JSON output. code=${code} stderr=${stderr.trim()} stdout=${trimmed.slice(0, 500)}`,
            ),
          );
        }
      }

      if (code !== 0) {
        if (parsed) {
          return resolve({
            parsed,
            stdout: trimmed,
            stderr: stderr.trim(),
            exitCode: code,
          });
        }
        const message = stderr.trim() || `Planner bridge exited with code ${code}`;
        return reject(new Error(message));
      }

      resolve({ parsed, stdout: trimmed, stderr: stderr.trim(), exitCode: code });
    });

    child.stdin.write(JSON.stringify(payload));
    child.stdin.end();
  });
}

const app = express();
app.use(express.json({ limit: '50mb' }));

app.get('/api/design/planning/health', async (_req, res) => {
  try {
    const { parsed, stderr } = await runBridge({ action: 'health' }, 45000);
    return res.json({
      ...parsed,
      apiVersion,
      sidecar: {
        pythonBin,
        bridgeScript,
      },
      stderr,
    });
  } catch (error) {
    console.error('[planner-sidecar] health failed:', error);
    return res.status(500).json({
      ok: false,
      apiVersion,
      error: error instanceof Error ? error.message : String(error),
      sidecar: {
        pythonBin,
        bridgeScript,
      },
    });
  }
});

app.post('/api/design/planning/run', async (req, res) => {
  try {
    const payload = {
      action: 'run',
      ...req.body,
    };
    const { parsed, stderr, exitCode } = await runBridge(payload, 600000);
    if (!parsed?.ok) {
      return res.status(400).json({
        ok: false,
        ...parsed,
        stderr,
        exitCode,
      });
    }
    return res.json({
      ...parsed,
      stderr,
      exitCode,
    });
  } catch (error) {
    console.error('[planner-sidecar] run failed:', error);
    return res.status(500).json({
      ok: false,
      apiVersion,
      error: error instanceof Error ? error.message : String(error),
    });
  }
});

app.listen(port, () => {
  console.log(`[planner-sidecar] version=${apiVersion}`);
  console.log(`[planner-sidecar] listening on http://127.0.0.1:${port}`);
  console.log(`[planner-sidecar] python=${pythonBin}`);
});

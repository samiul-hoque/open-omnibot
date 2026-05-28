// ============================================
// DataLogger — targeted unit tests
// ============================================

import { describe, it, before, after } from 'node:test';
import assert from 'node:assert';
import { mkdtempSync, rmSync, existsSync, readFileSync, readdirSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

import { DataLogger } from '../../src/logging/dataLogger.js';
import { config } from '../../src/config.js';

describe('DataLogger.markImuStuck', () => {
    let tmp;
    let origDir;
    let origEnabled;

    before(() => {
        tmp = mkdtempSync(join(tmpdir(), 'datalogger-test-'));
        origDir = config.logging.directory;
        origEnabled = config.logging.enabled;
        config.logging.directory = tmp;
        config.logging.enabled = true;
    });

    after(() => {
        config.logging.directory = origDir;
        config.logging.enabled = origEnabled;
        rmSync(tmp, { recursive: true, force: true });
    });

    it('sets imu_stuck_during_session and timestamp on first call', () => {
        const logger = new DataLogger();
        logger.start({ prefix: 'test_mark_imu_stuck_first' });

        logger.markImuStuck();

        assert.ok(existsSync(logger.metaFilename), 'metadata sidecar exists');
        const meta = JSON.parse(readFileSync(logger.metaFilename, 'utf8'));
        assert.strictEqual(meta.imu_stuck_during_session, true);
        assert.ok(typeof meta.imu_stuck_first_at === 'string');
        assert.doesNotThrow(() => new Date(meta.imu_stuck_first_at).toISOString());

        logger.stop();
    });

    it('is idempotent — subsequent calls do not overwrite timestamp', async () => {
        const logger = new DataLogger();
        logger.start({ prefix: 'test_mark_imu_stuck_idempotent' });

        logger.markImuStuck();
        const meta1 = JSON.parse(readFileSync(logger.metaFilename, 'utf8'));
        const firstAt = meta1.imu_stuck_first_at;

        // Small delay to guarantee a different ISO string if the method
        // erroneously rewrote the timestamp.
        await new Promise(r => setTimeout(r, 5));
        logger.markImuStuck();

        const meta2 = JSON.parse(readFileSync(logger.metaFilename, 'utf8'));
        assert.strictEqual(meta2.imu_stuck_during_session, true);
        assert.strictEqual(meta2.imu_stuck_first_at, firstAt, 'timestamp frozen after first call');

        logger.stop();
    });

    it('is a no-op when logger is stopped (no active session)', () => {
        const logger = new DataLogger();
        // No start() called — _meta is null.
        assert.doesNotThrow(() => logger.markImuStuck());
    });

    it('persists across stop → written into the final metadata sidecar', () => {
        const logger = new DataLogger();
        logger.start({ prefix: 'test_mark_imu_stuck_persists' });
        logger.markImuStuck();
        logger.stop();

        // logger.stop() finalizes the sidecar. Re-read from the filename
        // captured before stop by reconstructing the path.
        const files = readdirSync(tmp)
            .filter(f => f.startsWith('test_mark_imu_stuck_persists') && f.endsWith('.meta.json'));
        assert.strictEqual(files.length, 1, 'exactly one metadata sidecar written');
        const meta = JSON.parse(readFileSync(join(tmp, files[0]), 'utf8'));
        assert.strictEqual(meta.imu_stuck_during_session, true);
        assert.ok(meta.endTime, 'finalized with endTime');
    });
});

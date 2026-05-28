#!/usr/bin/env node
// ============================================
// UWB Debug Script
// ============================================
// Interactive terminal for debugging DWM1001 UWB listener.
// Usage: node scripts/uwb-debug.js [COM_PORT] [BAUD_RATE]
//
// Commands:
//   Type anything and press Enter to send to the device
//   Ctrl+C to exit
//
// Quick commands:
//   Just press Enter twice, then type "lep" to start position stream
//   Press Enter to stop the stream
//

import { SerialPort } from 'serialport';
import readline from 'readline';

const portPath = process.argv[2] || 'COM13';
const baudRate = parseInt(process.argv[3]) || 115200;

console.log('='.repeat(60));
console.log('  UWB Debug Terminal');
console.log('='.repeat(60));
console.log(`Port: ${portPath}`);
console.log(`Baud: ${baudRate}`);
console.log('');
console.log('Special commands (type these):');
console.log('  !raw        - Toggle raw byte display');
console.log('  !dtr        - Reset device via DTR toggle (like terminal connect)');
console.log('  !init       - Enter shell mode and start lep stream');
console.log('  !les        - Enter shell mode and start les stream');
console.log('  !info       - Get system info (si command)');
console.log('  !mode       - Check current node mode (nmg)');
console.log('  !passive    - Set node to passive mode (nmp)');
console.log('  !stop       - Send Enter to stop stream');
console.log('  !reset      - Send break signal');
console.log('  !binary     - Try binary API mode query');
console.log('');
console.log('DWM1001 commands:');
console.log('  [Enter]     - Wake up / stop stream / repeat last cmd');
console.log('  lep         - Start position stream (CSV)');
console.log('  les         - Start distance + position stream');
console.log('  si          - Show system info');
console.log('  nmg         - Get current node mode');
console.log('  nmp         - Set node to PASSIVE mode (listener)');
console.log('  la          - Show anchor list');
console.log('  ?           - Show all commands');
console.log('  quit        - Exit shell mode');
console.log('='.repeat(60));
console.log('');

// List available ports first
console.log('Available serial ports:');
const ports = await SerialPort.list();
for (const p of ports) {
    const marker = p.path === portPath ? ' <-- selected' : '';
    console.log(`  ${p.path}: ${p.manufacturer || p.pnpId || 'Unknown'}${marker}`);
}
console.log('');

// Open the port
console.log(`Opening ${portPath}...`);
const port = new SerialPort({
    path: portPath,
    baudRate: baudRate,
    autoOpen: false,
});

port.on('error', (err) => {
    console.error(`\n[ERROR] ${err.message}`);
    process.exit(1);
});

port.on('close', () => {
    console.log('\n[Port closed]');
    process.exit(0);
});

// Buffer for incoming data
let buffer = '';
let showRaw = false;
let bytesReceived = 0;

port.on('data', (data) => {
    bytesReceived += data.length;

    if (showRaw) {
        // Show raw hex bytes
        const hex = [...data].map(b => b.toString(16).padStart(2, '0')).join(' ');
        const ascii = [...data].map(b => (b >= 32 && b < 127) ? String.fromCharCode(b) : '.').join('');
        console.log(`\x1b[90m[RAW ${data.length}B] ${hex}  |${ascii}|\x1b[0m`);
    }

    const text = data.toString();
    buffer += text;

    // Print each complete line
    const lines = buffer.split('\n');
    buffer = lines.pop(); // Keep incomplete line in buffer

    for (const line of lines) {
        const trimmed = line.trim();
        if (trimmed) {
            // Color code different message types
            if (trimmed.startsWith('POS')) {
                console.log(`\x1b[32m${trimmed}\x1b[0m`); // Green for position
            } else if (trimmed.startsWith('DIST')) {
                console.log(`\x1b[36m${trimmed}\x1b[0m`); // Cyan for distance
            } else if (trimmed.includes('dwm>')) {
                console.log(`\x1b[33m${trimmed}\x1b[0m`); // Yellow for prompt
            } else if (trimmed.includes('error') || trimmed.includes('Error')) {
                console.log(`\x1b[31m${trimmed}\x1b[0m`); // Red for errors
            } else {
                console.log(trimmed);
            }
        }
    }
});

// Helper to send data with logging
function sendData(data, description) {
    port.write(data, (err) => {
        if (err) {
            console.error(`[Write error] ${err.message}`);
        } else {
            const escaped = data.replace(/\r/g, '\\r').replace(/\n/g, '\\n');
            console.log(`\x1b[36m[Sent: "${escaped}"] ${description || ''}\x1b[0m`);
        }
    });
}

// Full init sequence (per MDEK1001 manual section 7.1)
async function runInitSequence() {
    console.log('\x1b[33m[Running init sequence (per manual)...]\x1b[0m');

    console.log('[Step 1/3] Sending first Enter (enter shell mode)...');
    sendData('\r', '(first enter)');
    await new Promise(r => setTimeout(r, 100));

    console.log('[Step 2/3] Sending second Enter (confirm shell mode)...');
    sendData('\r', '(second enter)');
    await new Promise(r => setTimeout(r, 500));

    console.log('[Step 3/3] Sending "lep" command (position stream)...');
    sendData('lep\r', '(start position stream)');

    console.log('\x1b[33m[Init complete - watch for "dwm>" prompt and POS messages]\x1b[0m');
    console.log('\x1b[33m[Tip: Press Enter to repeat last command or stop stream]\x1b[0m');
}

// Reset device via DTR toggle (like terminal programs do on connect)
async function resetViaDtr() {
    console.log('\x1b[33m[Resetting device via DTR toggle...]\x1b[0m');

    // Toggle DTR off then on - this resets many USB serial devices
    console.log('[Step 1/4] DTR off...');
    port.set({ dtr: false });
    await new Promise(r => setTimeout(r, 100));

    console.log('[Step 2/4] DTR on (device should reset)...');
    port.set({ dtr: true });

    console.log('[Step 3/4] Waiting 1s for device boot...');
    await new Promise(r => setTimeout(r, 1000));

    console.log('[Step 4/4] Sending Enter twice to enter shell mode...');
    sendData('\r', '(first enter)');
    await new Promise(r => setTimeout(r, 50));
    sendData('\r', '(second enter)');
    await new Promise(r => setTimeout(r, 300));

    console.log('\x1b[33m[Reset complete - try !init or type commands]\x1b[0m');
}

// Show les stream (includes anchor ranges)
async function runLesStream() {
    console.log('\x1b[33m[Starting les stream (shows ranges + position)...]\x1b[0m');

    sendData('\r', '(enter shell)');
    await new Promise(r => setTimeout(r, 100));
    sendData('\r', '(confirm)');
    await new Promise(r => setTimeout(r, 500));
    sendData('les\r', '(start les stream)');

    console.log('\x1b[33m[les stream started - press Enter to stop]\x1b[0m');
}

// Binary mode query - try to communicate in API mode
async function runBinaryQuery() {
    console.log('\x1b[33m[Trying binary API mode query...]\x1b[0m');

    // DWM1001 binary TLV: Type 0x02 = dwm_pos_get
    // Frame format: [type:1][length:1][value:n]
    // For dwm_pos_get: type=0x02, length=0x00
    const posGetCmd = Buffer.from([0x02, 0x00]);

    port.write(posGetCmd, (err) => {
        if (err) {
            console.error(`[Write error] ${err.message}`);
        } else {
            console.log('\x1b[36m[Sent binary: 02 00 (dwm_pos_get)]\x1b[0m');
        }
    });

    console.log('\x1b[33m[Binary query sent - watch for response with !raw]\x1b[0m');
}

// Open the port
await new Promise((resolve, reject) => {
    port.open((err) => {
        if (err) reject(err);
        else resolve();
    });
});

console.log('[Port opened successfully]');
console.log('[Waiting for device... (boot message may take a few seconds)]');
console.log('');

// Create readline interface for user input
const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout,
    prompt: '> ',
});

rl.on('line', async (line) => {
    // Handle special commands
    if (line === '!raw') {
        showRaw = !showRaw;
        console.log(`\x1b[33m[Raw mode: ${showRaw ? 'ON' : 'OFF'}]\x1b[0m`);
        rl.prompt();
        return;
    }

    if (line === '!reset') {
        console.log('\x1b[33m[Sending break signal...]\x1b[0m');
        port.set({ brk: true });
        await new Promise(r => setTimeout(r, 100));
        port.set({ brk: false });
        console.log('\x1b[33m[Break signal sent]\x1b[0m');
        rl.prompt();
        return;
    }

    if (line === '!dtr') {
        await resetViaDtr();
        rl.prompt();
        return;
    }

    if (line === '!init') {
        await runInitSequence();
        rl.prompt();
        return;
    }

    if (line === '!les') {
        await runLesStream();
        rl.prompt();
        return;
    }

    if (line === '!binary') {
        await runBinaryQuery();
        rl.prompt();
        return;
    }

    if (line === '!mode') {
        console.log('\x1b[33m[Checking node mode...]\x1b[0m');
        sendData('\r', '(enter shell)');
        await new Promise(r => setTimeout(r, 100));
        sendData('\r', '(confirm)');
        await new Promise(r => setTimeout(r, 300));
        sendData('nmg\r', '(get node mode)');
        rl.prompt();
        return;
    }

    if (line === '!passive') {
        console.log('\x1b[33m[Setting node to PASSIVE mode...]\x1b[0m');
        sendData('\r', '(enter shell)');
        await new Promise(r => setTimeout(r, 100));
        sendData('\r', '(confirm)');
        await new Promise(r => setTimeout(r, 300));
        sendData('nmp\r', '(set passive mode)');
        console.log('\x1b[33m[Note: Device may reboot after mode change]\x1b[0m');
        rl.prompt();
        return;
    }

    if (line === '!info') {
        console.log('\x1b[33m[Getting system info...]\x1b[0m');
        sendData('\r', '(enter shell)');
        await new Promise(r => setTimeout(r, 100));
        sendData('\r', '(confirm)');
        await new Promise(r => setTimeout(r, 300));
        sendData('si\r', '(system info)');
        rl.prompt();
        return;
    }

    if (line === '!stop') {
        sendData('\r\n', '(stop stream)');
        rl.prompt();
        return;
    }

    if (line === '!status') {
        console.log(`\x1b[33m[Bytes received: ${bytesReceived}]\x1b[0m`);
        rl.prompt();
        return;
    }

    // Send the line with \r\n
    sendData(line + '\r\n');
    rl.prompt();
});

rl.on('close', () => {
    console.log('\n[Closing...]');
    port.close();
});

// Handle Ctrl+C
process.on('SIGINT', () => {
    console.log('\n[Interrupted]');
    rl.close();
});

// Show prompt
rl.prompt();

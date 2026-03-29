const GameMode = {
    IDLE: 'idle',
    DRAGGING: 'dragging',
    PLACING_NODE: 'placing',
    ANIMATING_AI: 'animating_ai'
};

let gameState = {
    mode: GameMode.IDLE,
    nodes: [],
    lines: [],
    activeNodes: new Set(),
    currentPlayer: 0,
    dragPath: [],
    lockedPath: [],
    placementDot: null,
    dragStartNode: null,
    dragEndNode: null,
    hoveredNode: null,
    animatingLine: null,
    animationProgress: 0,
    gameOver: false,
    reviewing: false,
    // Track last input type per-event instead of a permanent flag,
    // so hybrid devices (Surface, iPad+keyboard) can use both.
    lastInputWasTouch: false
};

let lastNodeCount = 4;
let canvas, ctx;
// Use smaller internal grid on mobile to save memory (~4MB → ~1MB) and speed up
// skeleton generation. CSS scaling handles display resolution.
const BOARD_SIZE = (window.innerWidth <= 768 || window.innerHeight <= 768) ? 500 : 1000;
let worker = null;
let pendingCallbacks = new Map();
let callbackId = 0;
let thinkingInterval = null;
let placingModeEnteredAt = 0;

function workerCall(type, data = {}) {
    return new Promise((resolve, reject) => {
        callbackId = (callbackId + 1) % Number.MAX_SAFE_INTEGER;
        const id = callbackId;
        const timeout = setTimeout(() => {
            pendingCallbacks.delete(id);
            reject(new Error(`Worker call '${type}' timed out`));
        }, 30000);
        pendingCallbacks.set(id, { resolve, reject, timeout });
        worker.postMessage({ type, data, id });
    });
}

function handleWorkerMessage(e) {
    const { id, error, ...payload } = e.data;
    if (pendingCallbacks.has(id)) {
        const { resolve, reject, timeout } = pendingCallbacks.get(id);
        clearTimeout(timeout);
        pendingCallbacks.delete(id);
        if (error) {
            reject(new Error(error));
        } else {
            resolve(payload);
        }
    }
}

function startThinking() {
    let dots = 0;
    setStatus('AI is thinking');
    thinkingInterval = setInterval(() => {
        dots = (dots + 1) % 4;
        setStatus('AI is thinking' + '.'.repeat(dots));
    }, 400);
}

function stopThinking() {
    if (thinkingInterval) {
        clearInterval(thinkingInterval);
        thinkingInterval = null;
    }
}

function updateUndoButton() {
    const btn = document.getElementById('undoBtn');
    btn.disabled = gameState.lines.length === 0 || gameState.mode !== GameMode.IDLE;
}

function isTouchActive() {
    return gameState.lastInputWasTouch;
}

async function init() {
    canvas = document.getElementById('canvas');
    canvas.width = BOARD_SIZE;
    canvas.height = BOARD_SIZE;
    ctx = canvas.getContext('2d');

    worker = new Worker('./ai-worker.js', { type: 'module' });
    worker.onmessage = handleWorkerMessage;
    worker.onerror = (e) => {
        setStatus('Failed to load game engine', 'lose');
    };

    // Mouse events — always active, but ignored if the most recent
    // pointerdown was a touch (prevents ghost clicks on touch devices).
    canvas.addEventListener('mousedown', (e) => {
        if (gameState.lastInputWasTouch) return;
        handleStart(e.clientX, e.clientY);
    });
    canvas.addEventListener('mousemove', (e) => {
        if (gameState.lastInputWasTouch) return;
        handleMove(e.clientX, e.clientY);
    });
    canvas.addEventListener('mouseup', (e) => {
        if (gameState.lastInputWasTouch) return;
        handleEnd(e.clientX, e.clientY);
    });
    canvas.addEventListener('click', (e) => {
        if (gameState.lastInputWasTouch) return;
        handleDesktopClick(e.clientX, e.clientY);
    });
    canvas.addEventListener('contextmenu', (e) => {
        if (gameState.mode === GameMode.PLACING_NODE) {
            e.preventDefault();
            cancelPlacement();
        }
    });
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && gameState.mode === GameMode.PLACING_NODE) {
            cancelPlacement();
        }
    });

    // Touch events — set flag per-gesture so mouse events from the
    // same tap are suppressed, but a subsequent mouse action works.
    canvas.addEventListener('touchstart', (e) => {
        gameState.lastInputWasTouch = true;
        e.preventDefault();
        handleStart(e.touches[0].clientX, e.touches[0].clientY);
    }, { passive: false });
    canvas.addEventListener('touchmove', (e) => {
        e.preventDefault();
        handleMove(e.touches[0].clientX, e.touches[0].clientY);
    }, { passive: false });
    canvas.addEventListener('touchend', (e) => {
        e.preventDefault();
        handleEnd(e.changedTouches[0].clientX, e.changedTouches[0].clientY);
        // Reset after a short delay so subsequent mouse events work
        // on hybrid devices (the browser fires mouse events ~300ms
        // after touchend).
        setTimeout(() => { gameState.lastInputWasTouch = false; }, 400);
    }, { passive: false });

    requestAnimationFrame(render);
    try {
        await workerCall('init', { nodeCount: 4, boardSize: BOARD_SIZE });
        await updateGameState();
        setStatus('Draw a line between two nodes');
    } catch (e) {
        setStatus('Failed to initialize: ' + e.message, 'lose');
    }
}

function getCanvasCoords(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    return {
        x: (clientX - rect.left) * (canvas.width / rect.width),
        y: (clientY - rect.top) * (canvas.height / rect.height)
    };
}

function handleStart(clientX, clientY) {
    if (gameState.reviewing) {
        gameState.reviewing = false;
        document.getElementById('gameOverModal').classList.add('visible');
        return;
    }
    if (gameState.mode !== GameMode.IDLE || gameState.gameOver) return;
    const { x, y } = getCanvasCoords(clientX, clientY);
    const node = findNodeAt(x, y);
    if (node && gameState.activeNodes.has(node.id)) {
        gameState.mode = GameMode.DRAGGING;
        gameState.dragStartNode = node.id;
        gameState.dragPath = [{ x: node.position.x, y: node.position.y }];
    }
}

async function handleMove(clientX, clientY) {
    const { x, y } = getCanvasCoords(clientX, clientY);
    const node = findNodeAt(x, y);
    gameState.hoveredNode = node ? node.id : null;

    if (gameState.mode === GameMode.DRAGGING) {
        const last = gameState.dragPath[gameState.dragPath.length - 1];
        if (Math.hypot(x - last.x, y - last.y) > 5) {
            gameState.dragPath.push({ x, y });
            gameState.dragPath = healPath(gameState.dragPath);
        }
    } else if (gameState.mode === GameMode.PLACING_NODE) {
        const pathData = gameState.lockedPath.flatMap(p => [p.x, p.y]);
        const { point } = await workerCall('get_closest_point', { pathData, x, y });
        const closest = { x: point[0], y: point[1] };

        const snapDist = isTouchActive() ? 80 : 30;
        if (Math.hypot(x - closest.x, y - closest.y) < snapDist) {
            const { valid } = await workerCall('validate_placement', {
                pathData, newNodeX: closest.x, newNodeY: closest.y
            });
            gameState.placementDot = { position: closest, valid };
        } else {
            gameState.placementDot = null;
        }
    }
}

function handleEnd(clientX, clientY) {
    if (gameState.mode !== GameMode.DRAGGING) return;
    const { x, y } = getCanvasCoords(clientX, clientY);
    let endNode = findNodeAt(x, y);

    // Snap to nearest active node if release was close but not on target
    if (!endNode || !gameState.activeNodes.has(endNode.id)) {
        const snapRange = isTouchActive() ? 60 : 30;
        let bestDist = snapRange;
        let bestNode = null;
        for (const node of gameState.nodes) {
            if (!gameState.activeNodes.has(node.id)) continue;
            const dist = Math.hypot(x - node.position.x, y - node.position.y);
            if (dist < bestDist) {
                bestDist = dist;
                bestNode = node;
            }
        }
        if (bestNode) endNode = bestNode;
    }

    if (!endNode || !gameState.activeNodes.has(endNode.id)) {
        gameState.mode = GameMode.IDLE;
        gameState.dragPath = [];
        setStatus('Release near a node to connect');
        return;
    }

    gameState.lockedPath = [...gameState.dragPath];
    gameState.lockedPath[gameState.lockedPath.length - 1] = { x: endNode.position.x, y: endNode.position.y };
    gameState.dragEndNode = endNode.id;
    gameState.mode = GameMode.PLACING_NODE;
    placingModeEnteredAt = performance.now();

    if (isTouchActive()) {
        document.getElementById('mobileOverlay').classList.add('visible');
        setStatus('Drag along curve to place node');
    } else {
        setStatus('Click on the curve to place a new node (ESC to cancel)', 'win');
    }
}

async function handleDesktopClick(clientX, clientY) {
    if (gameState.mode !== GameMode.PLACING_NODE || isTouchActive()) return;

    // Ignore the click event that fires as part of the drag-release gesture.
    if (performance.now() - placingModeEnteredAt < 300) return;

    const { x, y } = getCanvasCoords(clientX, clientY);
    const pathData = gameState.lockedPath.flatMap(p => [p.x, p.y]);
    const { point } = await workerCall('get_closest_point', { pathData, x, y });
    const closest = { x: point[0], y: point[1] };

    if (Math.hypot(x - closest.x, y - closest.y) < 40) {
        const { valid } = await workerCall('validate_placement', {
            pathData, newNodeX: closest.x, newNodeY: closest.y
        });
        if (valid) {
            gameState.placementDot = { position: closest, valid: true };
            await executeMove();
            return;
        }
    }

    // Fallback: use the placement dot from the last mousemove if valid
    if (gameState.placementDot?.valid) {
        await executeMove();
    }
}

window.confirmMove = async () => {
    if (isTouchActive()) await executeMove();
};

async function executeMove() {
    if (!gameState.placementDot || !gameState.placementDot.valid) {
        setStatus('Select a valid spot on the line', 'lose');
        return;
    }
    const pathData = gameState.lockedPath.flatMap(p => [p.x, p.y]);
    const newPos = gameState.placementDot.position;
    document.getElementById('mobileOverlay').classList.remove('visible');

    const { success } = await workerCall('apply_move', {
        fromNode: gameState.dragStartNode, toNode: gameState.dragEndNode,
        pathData, newNodeX: newPos.x, newNodeY: newPos.y
    });

    if (!success) {
        // Fetch the specific reason the move was rejected
        try {
            const { moveError } = await workerCall('get_move_error', {
                fromNode: gameState.dragStartNode, toNode: gameState.dragEndNode,
                pathData, newNodeX: newPos.x, newNodeY: newPos.y
            });
            setStatus(moveError || 'Move rejected — try a different path', 'lose');
        } catch {
            setStatus('Move rejected — try a different path', 'lose');
        }
        gameState.mode = GameMode.IDLE;
        gameState.dragPath = [];
        gameState.lockedPath = [];
        gameState.placementDot = null;
        return;
    }

    await updateGameState();
    gameState.mode = GameMode.IDLE;
    gameState.dragPath = [];
    gameState.lockedPath = [];
    gameState.placementDot = null;
    updateUndoButton();

    if (!gameState.gameOver) {
        startThinking();
        performAIMove();
    }
}

window.cancelPlacement = () => {
    gameState.mode = GameMode.IDLE;
    gameState.dragPath = [];
    gameState.lockedPath = [];
    gameState.placementDot = null;
    document.getElementById('mobileOverlay').classList.remove('visible');
    setStatus('Move cancelled');
};

function healPath(points) {
    if (points.length < 4) return points;
    const healed = [points[0]];
    for (let i = 1; i < points.length; i++) {
        const current = points[i];
        for (let j = 0; j < healed.length - 2; j++) {
            if (segmentsIntersect(healed[healed.length - 1], current, healed[j], healed[j + 1])) {
                return healPath([...healed.slice(0, j + 2), current]);
            }
        }
        healed.push(current);
    }
    return healed;
}

function segmentsIntersect(a1, a2, b1, b2) {
    const ccw = (a, b, c) => (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x);
    return ccw(a1, b1, b2) !== ccw(a2, b1, b2) && ccw(a1, a2, b1) !== ccw(a1, a2, b2);
}

async function updateGameState() {
    const { state } = await workerCall('get_state');
    let idx = 0;
    const nodeCount = state[idx++];
    const newNodes = [];
    for (let i = 0; i < nodeCount; i++) {
        newNodes.push({
            id: state[idx++],
            position: { x: state[idx++], y: state[idx++] },
            connectionCount: state[idx++]
        });
    }
    const lineCount = state[idx++];
    const newLines = [];
    for (let i = 0; i < lineCount; i++) {
        const id = state[idx++];
        const from = state[idx++];
        const to = state[idx++];
        const pointCount = state[idx++];
        const polyline = [];
        for (let j = 0; j < pointCount; j++) polyline.push({ x: state[idx++], y: state[idx++] });
        const newNodePos = { x: state[idx++], y: state[idx++] };
        const player = state[idx++];
        newLines.push({ id, from, to, polyline, newNodePos, player });
    }
    const newCurrentPlayer = state[idx++];

    const [{ activeNodes }, { gameOver }] = await Promise.all([
        workerCall('get_active_nodes'),
        workerCall('is_game_over'),
    ]);

    // Atomic update
    gameState.nodes = newNodes;
    gameState.lines = newLines;
    gameState.currentPlayer = newCurrentPlayer;
    gameState.activeNodes = new Set(activeNodes);

    if (gameOver) {
        gameState.gameOver = true;
        showGameOver();
    }
    updateUndoButton();
}

async function performAIMove() {
    try {
        const { move: aiMoveData } = await workerCall('get_ai_move');
        stopThinking();
        if (aiMoveData.length === 0) {
            gameState.gameOver = true;
            showGameOver();
            return;
        }
        let idx = 0;
        const fromNode = aiMoveData[idx++];
        const toNode = aiMoveData[idx++];
        const pointCount = aiMoveData[idx++];
        const path = [];
        for (let i = 0; i < pointCount; i++) path.push({ x: aiMoveData[idx++], y: aiMoveData[idx++] });
        const newPos = { x: aiMoveData[idx++], y: aiMoveData[idx++] };
        gameState.mode = GameMode.ANIMATING_AI;
        gameState.animatingLine = { path, newPos };
        gameState.animationProgress = 0;
        const startTime = performance.now();
        function animate(currentTime) {
            gameState.animationProgress = Math.min((currentTime - startTime) / 500, 1.0);
            if (gameState.animationProgress < 1.0) requestAnimationFrame(animate);
            else finishAIMove(fromNode, toNode, path, newPos);
        }
        requestAnimationFrame(animate);
    } catch (e) {
        stopThinking();
        setStatus('AI error: ' + e.message, 'lose');
        gameState.mode = GameMode.IDLE;
    }
}

async function finishAIMove(fromNode, toNode, path, newPos) {
    const { success } = await workerCall('apply_move', {
        fromNode, toNode, pathData: path.flatMap(p => [p.x, p.y]),
        newNodeX: newPos.x, newNodeY: newPos.y
    });
    if (!success) {
        gameState.mode = GameMode.IDLE;
        gameState.animatingLine = null;
        setStatus('Your turn!');
        return;
    }
    await updateGameState();
    gameState.mode = GameMode.IDLE;
    gameState.animatingLine = null;
    updateUndoButton();
    if (!gameState.gameOver) setStatus('Your turn!');
}

function findNodeAt(x, y) {
    const threshold = isTouchActive() ? 30 : 15;
    let closest = null;
    let minDist = threshold;
    for (const n of gameState.nodes) {
        const d = Math.hypot(x - n.position.x, y - n.position.y);
        if (d < minDist) {
            minDist = d;
            closest = n;
        }
    }
    return closest;
}

// Catmull-Rom spline: passes through every point
function drawSmoothPath(ctx, points) {
    if (!points || points.length < 2) return;
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    if (points.length === 2) {
        ctx.lineTo(points[1].x, points[1].y);
    } else {
        for (let i = 0; i < points.length - 1; i++) {
            const p0 = points[Math.max(i - 1, 0)];
            const p1 = points[i];
            const p2 = points[i + 1];
            const p3 = points[Math.min(i + 2, points.length - 1)];
            const t = 6; // standard Catmull-Rom tension; lower = rounder curves
            const cp1x = p1.x + (p2.x - p0.x) / t;
            const cp1y = p1.y + (p2.y - p0.y) / t;
            const cp2x = p2.x - (p3.x - p1.x) / t;
            const cp2y = p2.y - (p3.y - p1.y) / t;
            ctx.bezierCurveTo(cp1x, cp1y, cp2x, cp2y, p2.x, p2.y);
        }
    }
    ctx.stroke();
}

function render() {
    ctx.fillStyle = '#1A1A1A';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Draw lines
    for (const line of gameState.lines) {
        ctx.strokeStyle = line.player === 0 ? '#60A5FA' : '#EF4444';
        ctx.lineWidth = 3;
        drawSmoothPath(ctx, line.polyline);
    }

    // Draw active path (dragging or locked)
    const activePath = gameState.mode === GameMode.DRAGGING ? gameState.dragPath :
                       gameState.mode === GameMode.PLACING_NODE ? gameState.lockedPath : null;
    if (activePath && activePath.length > 1) {
        ctx.strokeStyle = '#3B82F6';
        ctx.lineWidth = 3;
        drawSmoothPath(ctx, activePath);
    }

    // Draw placement dot
    if (gameState.mode === GameMode.PLACING_NODE && gameState.placementDot) {
        ctx.fillStyle = gameState.placementDot.valid ? '#10B981' : '#EF4444';
        ctx.beginPath();
        ctx.arc(gameState.placementDot.position.x, gameState.placementDot.position.y, 8, 0, Math.PI * 2);
        ctx.fill();
    }

    // Draw nodes with remaining connection count
    for (const node of gameState.nodes) {
        const isActive = gameState.activeNodes.has(node.id);
        const remaining = 3 - node.connectionCount;

        ctx.fillStyle = isActive ? '#3B82F6' : '#555';
        ctx.strokeStyle = '#FFF';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(node.position.x, node.position.y, 8, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        ctx.fillStyle = '#FFF';
        ctx.font = 'bold 10px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(String(remaining), node.position.x, node.position.y + 0.5);
    }

    // Draw AI move animation
    if (gameState.mode === GameMode.ANIMATING_AI && gameState.animatingLine) {
        const anim = gameState.animatingLine;
        const len = Math.floor(gameState.animationProgress * anim.path.length);
        if (len > 1) {
            ctx.strokeStyle = '#EF4444'; ctx.lineWidth = 3;
            drawSmoothPath(ctx, anim.path.slice(0, len));
        }
    }
    requestAnimationFrame(render);
}

function setStatus(msg, type = '') {
    const el = document.getElementById('status');
    el.textContent = msg; el.className = type;
}

function showGameOver() {
    stopThinking();
    const humanWins = gameState.currentPlayer === 1;
    setStatus(humanWins ? 'You win!' : 'AI wins', humanWins ? 'win' : 'lose');
    document.getElementById('winnerText').textContent = humanWins ? 'YOU WIN' : 'AI WINS';
    document.getElementById('winnerText').style.color = humanWins ? '#10B981' : '#EF4444';
    document.getElementById('gameOverModal').classList.add('visible');
}

function closeModal() {
    document.getElementById('gameOverModal').classList.remove('visible');
}

window.reviewBoard = () => {
    gameState.reviewing = true;
    closeModal();
};

window.newGameFromModal = () => {
    window.newGame(lastNodeCount);
};

window.undoMove = async () => {
    if (gameState.mode !== GameMode.IDLE || thinkingInterval !== null) return;
    let undone = false;
    for (let i = 0; i < 2; i++) {
        const { success } = await workerCall('undo');
        if (!success) break;
        undone = true;
    }
    if (undone) {
        gameState.gameOver = false;
        gameState.reviewing = false;
        closeModal();
        await updateGameState();
        setStatus('Move undone — your turn');
    }
};

window.newGame = async (n) => {
    lastNodeCount = n;
    gameState.mode = GameMode.IDLE;
    gameState.gameOver = false;
    gameState.reviewing = false;
    gameState.nodes = [];
    gameState.lines = [];
    gameState.activeNodes = new Set();
    gameState.dragPath = [];
    gameState.lockedPath = [];
    gameState.placementDot = null;
    gameState.animatingLine = null;
    gameState.hoveredNode = null;
    gameState.dragStartNode = null;
    gameState.dragEndNode = null;
    stopThinking();
    setStatus('Initializing...');
    closeModal();

    try {
        await workerCall('init', { nodeCount: n, boardSize: BOARD_SIZE });
        await updateGameState();
        setStatus('Draw a line between two nodes');
    } catch (e) {
        setStatus('Failed to start game: ' + e.message, 'lose');
    }
};

window.closeModal = closeModal;
window.cancelPlacement = window.cancelPlacement;
window.confirmMove = window.confirmMove;
init();

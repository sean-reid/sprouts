let wasm;

function getArrayF64FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat64ArrayMemory0().subarray(ptr / 8, ptr / 8 + len);
}

let cachedFloat64ArrayMemory0 = null;
function getFloat64ArrayMemory0() {
    if (cachedFloat64ArrayMemory0 === null || cachedFloat64ArrayMemory0.byteLength === 0) {
        cachedFloat64ArrayMemory0 = new Float64Array(wasm.memory.buffer);
    }
    return cachedFloat64ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return decodeText(ptr, len);
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function passArrayF64ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 8, 8) >>> 0;
    getFloat64ArrayMemory0().set(arg, ptr / 8);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

let WASM_VECTOR_LEN = 0;

const SproutsGameFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_sproutsgame_free(ptr >>> 0, 1));

export class SproutsGame {
    static __wrap(ptr) {
        ptr = ptr >>> 0;
        const obj = Object.create(SproutsGame.prototype);
        obj.__wbg_ptr = ptr;
        SproutsGameFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        SproutsGameFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_sproutsgame_free(ptr, 0);
    }
    /**
     * @param {number} from_node
     * @param {number} to_node
     * @param {Float64Array} path_data
     * @param {number} new_node_x
     * @param {number} new_node_y
     * @returns {boolean}
     */
    apply_move(from_node, to_node, path_data, new_node_x, new_node_y) {
        const ptr0 = passArrayF64ToWasm0(path_data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.sproutsgame_apply_move(this.__wbg_ptr, from_node, to_node, ptr0, len0, new_node_x, new_node_y);
        return ret !== 0;
    }
    /**
     * @returns {Float64Array}
     */
    get_ai_move() {
        const ret = wasm.sproutsgame_get_ai_move(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * @returns {boolean}
     */
    is_game_over() {
        const ret = wasm.sproutsgame_is_game_over(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @param {number} from_node
     * @param {number} to_node
     * @param {Float64Array} path_data
     * @param {number} new_node_x
     * @param {number} new_node_y
     * @returns {string}
     */
    get_move_error(from_node, to_node, path_data, new_node_x, new_node_y) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ptr0 = passArrayF64ToWasm0(path_data, wasm.__wbindgen_malloc);
            const len0 = WASM_VECTOR_LEN;
            const ret = wasm.sproutsgame_get_move_error(this.__wbg_ptr, from_node, to_node, ptr0, len0, new_node_x, new_node_y);
            deferred2_0 = ret[0];
            deferred2_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * @returns {Float64Array}
     */
    get_active_nodes() {
        const ret = wasm.sproutsgame_get_active_nodes(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * @param {Float64Array} path_data
     * @param {number} new_node_x
     * @param {number} new_node_y
     * @returns {boolean}
     */
    validate_placement(path_data, new_node_x, new_node_y) {
        const ptr0 = passArrayF64ToWasm0(path_data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.sproutsgame_validate_placement(this.__wbg_ptr, ptr0, len0, new_node_x, new_node_y);
        return ret !== 0;
    }
    /**
     * @param {number} initial_nodes
     * @param {number} board_size
     * @returns {SproutsGame}
     */
    static new_with_board_size(initial_nodes, board_size) {
        const ret = wasm.sproutsgame_new_with_board_size(initial_nodes, board_size);
        return SproutsGame.__wrap(ret);
    }
    /**
     * @param {Float64Array} path_data
     * @param {number} target_x
     * @param {number} target_y
     * @returns {Float64Array}
     */
    get_closest_point_on_path(path_data, target_x, target_y) {
        const ptr0 = passArrayF64ToWasm0(path_data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.sproutsgame_get_closest_point_on_path(this.__wbg_ptr, ptr0, len0, target_x, target_y);
        var v2 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v2;
    }
    /**
     * @param {number} initial_nodes
     */
    constructor(initial_nodes) {
        const ret = wasm.sproutsgame_new(initial_nodes);
        this.__wbg_ptr = ret >>> 0;
        SproutsGameFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @returns {boolean}
     */
    undo() {
        const ret = wasm.sproutsgame_undo(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Get current game state as Float64Array.
     * Format: [node_count, ...nodes, line_count, ...lines, current_player]
     * @returns {Float64Array}
     */
    get_state() {
        const ret = wasm.sproutsgame_get_state(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
}
if (Symbol.dispose) SproutsGame.prototype[Symbol.dispose] = SproutsGame.prototype.free;

const EXPECTED_RESPONSE_TYPES = new Set(['basic', 'cors', 'default']);

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);
            } catch (e) {
                const validResponse = module.ok && EXPECTED_RESPONSE_TYPES.has(module.type);

                if (validResponse && module.headers.get('Content-Type') !== 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else {
                    throw e;
                }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);
    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };
        } else {
            return instance;
        }
    }
}

function __wbg_get_imports() {
    const imports = {};
    imports.wbg = {};
    imports.wbg.__wbg___wbindgen_throw_dd24417ed36fc46e = function(arg0, arg1) {
        throw new Error(getStringFromWasm0(arg0, arg1));
    };
    imports.wbg.__wbindgen_init_externref_table = function() {
        const table = wasm.__wbindgen_externrefs;
        const offset = table.grow(4);
        table.set(0, undefined);
        table.set(offset + 0, undefined);
        table.set(offset + 1, null);
        table.set(offset + 2, true);
        table.set(offset + 3, false);
    };

    return imports;
}

function __wbg_finalize_init(instance, module) {
    wasm = instance.exports;
    __wbg_init.__wbindgen_wasm_module = module;
    cachedFloat64ArrayMemory0 = null;
    cachedUint8ArrayMemory0 = null;


    wasm.__wbindgen_start();
    return wasm;
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (typeof module !== 'undefined') {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();
    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }
    const instance = new WebAssembly.Instance(module, imports);
    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;


    if (typeof module_or_path !== 'undefined') {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (typeof module_or_path === 'undefined') {
        module_or_path = new URL('sprouts_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync };
export default __wbg_init;

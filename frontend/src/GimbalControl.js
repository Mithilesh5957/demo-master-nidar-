import React, { useState, useEffect, useRef, useCallback } from 'react';
import axios from 'axios';

const IP_ADDRESS = "localhost";

const GimbalControl = ({ socket, scoutData }) => {
    // Gimbal state from telemetry
    const gimbalYaw = scoutData?.gimbal_yaw ?? 0;
    const gimbalPitch = scoutData?.gimbal_pitch ?? 0;
    const gimbalRoll = scoutData?.gimbal_roll ?? 0;

    // Absolute positioning
    const [targetPitch, setTargetPitch] = useState(0);
    const [targetYaw, setTargetYaw] = useState(0);

    // Mode
    const [gimbalMode, setGimbalMode] = useState('follow');
    const [isRecording, setIsRecording] = useState(false);

    // Joystick state
    const joystickRef = useRef(null);
    const isDragging = useRef(false);
    const rateInterval = useRef(null);
    const [joystickPos, setJoystickPos] = useState({ x: 0, y: 0 });

    // Keyboard gimbal control (I/K/J/L)
    useEffect(() => {
        const activeKeys = new Set();
        
        const sendRateFromKeys = () => {
            let pitchSpeed = 0;
            let yawSpeed = 0;
            if (activeKeys.has('i')) pitchSpeed = 30;
            if (activeKeys.has('k')) pitchSpeed = -30;
            if (activeKeys.has('j')) yawSpeed = -30;
            if (activeKeys.has('l')) yawSpeed = 30;

            if (pitchSpeed !== 0 || yawSpeed !== 0) {
                socket?.emit('gimbal_input', { pitch_speed: pitchSpeed, yaw_speed: yawSpeed, drone: 'scout' });
            }
        };

        const handleKeyDown = (e) => {
            const key = e.key.toLowerCase();
            if (['i', 'k', 'j', 'l'].includes(key)) {
                e.preventDefault();
                if (!activeKeys.has(key)) {
                    activeKeys.add(key);
                    sendRateFromKeys();
                }
            }
        };

        const handleKeyUp = (e) => {
            const key = e.key.toLowerCase();
            if (['i', 'k', 'j', 'l'].includes(key)) {
                activeKeys.delete(key);
                if (activeKeys.size === 0) {
                    axios.post(`http://${IP_ADDRESS}:5000/gimbal/stop`, { drone: 'scout' }).catch(() => {});
                } else {
                    sendRateFromKeys();
                }
            }
        };

        window.addEventListener('keydown', handleKeyDown);
        window.addEventListener('keyup', handleKeyUp);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
            window.removeEventListener('keyup', handleKeyUp);
        };
    }, [socket]);

    // --- Joystick Mouse Handlers ---
    const getJoystickOffset = useCallback((e, rect) => {
        const x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        const y = ((e.clientY - rect.top) / rect.height) * 2 - 1;
        const clamped = (v) => Math.max(-1, Math.min(1, v));
        return { x: clamped(x), y: clamped(y) };
    }, []);

    const handleJoystickDown = (e) => {
        isDragging.current = true;
        const rect = joystickRef.current.getBoundingClientRect();
        const pos = getJoystickOffset(e, rect);
        setJoystickPos(pos);

        // Start sending rates at 10Hz
        rateInterval.current = setInterval(() => {
            const yawSpeed = joystickPos.x * 50;  // ±50°/s max
            const pitchSpeed = -joystickPos.y * 50; // Inverted: drag up = pitch up
            socket?.emit('gimbal_input', { pitch_speed: pitchSpeed, yaw_speed: yawSpeed, drone: 'scout' });
        }, 100);
    };

    const handleJoystickMove = (e) => {
        if (!isDragging.current) return;
        const rect = joystickRef.current.getBoundingClientRect();
        const pos = getJoystickOffset(e, rect);
        setJoystickPos(pos);
    };

    const handleJoystickUp = () => {
        isDragging.current = false;
        setJoystickPos({ x: 0, y: 0 });
        if (rateInterval.current) {
            clearInterval(rateInterval.current);
            rateInterval.current = null;
        }
        axios.post(`http://${IP_ADDRESS}:5000/gimbal/stop`, { drone: 'scout' }).catch(() => {});
    };

    useEffect(() => {
        window.addEventListener('mouseup', handleJoystickUp);
        window.addEventListener('mousemove', handleJoystickMove);
        return () => {
            window.removeEventListener('mouseup', handleJoystickUp);
            window.removeEventListener('mousemove', handleJoystickMove);
            if (rateInterval.current) clearInterval(rateInterval.current);
        };
    }, []);

    // --- Command Helpers ---
    const sendGoto = () => {
        axios.post(`http://${IP_ADDRESS}:5000/gimbal/goto`, { pitch: targetPitch, yaw: targetYaw, drone: 'scout' }).catch(() => {});
    };

    const sendCenter = () => {
        axios.post(`http://${IP_ADDRESS}:5000/gimbal/center`, { drone: 'scout' }).catch(() => {});
        setTargetPitch(0);
        setTargetYaw(0);
    };

    const sendDown = () => {
        axios.post(`http://${IP_ADDRESS}:5000/gimbal/down`, { drone: 'scout' }).catch(() => {});
        setTargetPitch(-90);
    };

    const sendPhoto = () => {
        axios.post(`http://${IP_ADDRESS}:5000/gimbal/photo`, { drone: 'scout' }).catch(() => {});
    };

    const toggleRecord = () => {
        if (isRecording) {
            axios.post(`http://${IP_ADDRESS}:5000/gimbal/record/stop`, { drone: 'scout' }).catch(() => {});
        } else {
            axios.post(`http://${IP_ADDRESS}:5000/gimbal/record/start`, { drone: 'scout' }).catch(() => {});
        }
        setIsRecording(!isRecording);
    };

    const sendMode = (mode) => {
        setGimbalMode(mode);
        axios.post(`http://${IP_ADDRESS}:5000/gimbal/mode`, { mode, drone: 'scout' }).catch(() => {});
    };

    // --- Attitude Indicator ---
    const AttitudeGauge = ({ label, value, min, max, color }) => {
        const pct = ((value - min) / (max - min)) * 100;
        return (
            <div className="flex flex-col items-center">
                <span className="text-[10px] text-slate-400 font-bold uppercase">{label}</span>
                <div className="w-16 h-1.5 bg-slate-700 rounded-full overflow-hidden mt-1">
                    <div
                        className={`h-full rounded-full transition-all duration-200 ${color}`}
                        style={{ width: `${Math.max(2, pct)}%`, marginLeft: label === 'YAW' ? `${Math.max(0, pct - 2)}%` : undefined }}
                    />
                </div>
                <span className="text-xs text-white font-mono mt-0.5">{value.toFixed(1)}°</span>
            </div>
        );
    };

    return (
        <div className="bg-slate-800/80 backdrop-blur-sm rounded-xl border border-slate-700 p-4 mt-4">
            <div className="flex items-center justify-between mb-3">
                <h3 className="text-sm font-bold text-slate-100 flex items-center gap-2">
                    🎯 C12 GIMBAL CONTROL
                    <span className="text-[10px] px-1.5 py-0.5 bg-emerald-500/20 text-emerald-400 rounded font-normal">LIVE</span>
                </h3>
                <span className="text-[10px] text-slate-500">I/K Pitch • J/L Yaw</span>
            </div>

            <div className="grid grid-cols-3 gap-4">

                {/* LEFT: Attitude + Mode */}
                <div className="space-y-3">
                    {/* Live Attitude */}
                    <div className="bg-slate-900/60 rounded-lg p-3">
                        <div className="text-[10px] text-slate-500 font-bold mb-2">ATTITUDE</div>
                        <div className="flex justify-between">
                            <AttitudeGauge label="YAW" value={gimbalYaw} min={-180} max={180} color="bg-blue-500" />
                            <AttitudeGauge label="PITCH" value={gimbalPitch} min={-90} max={30} color="bg-emerald-500" />
                            <AttitudeGauge label="ROLL" value={gimbalRoll} min={-45} max={45} color="bg-amber-500" />
                        </div>
                    </div>

                    {/* Mode Toggle */}
                    <div className="flex gap-1">
                        <button
                            onClick={() => sendMode('follow')}
                            className={`flex-1 text-[10px] py-1.5 rounded font-bold transition-colors ${gimbalMode === 'follow' ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-400 hover:bg-slate-600'}`}
                        >
                            FOLLOW
                        </button>
                        <button
                            onClick={() => sendMode('lock')}
                            className={`flex-1 text-[10px] py-1.5 rounded font-bold transition-colors ${gimbalMode === 'lock' ? 'bg-orange-600 text-white' : 'bg-slate-700 text-slate-400 hover:bg-slate-600'}`}
                        >
                            LOCK
                        </button>
                    </div>
                </div>

                {/* CENTER: Virtual Joystick */}
                <div className="flex flex-col items-center">
                    <div className="text-[10px] text-slate-500 font-bold mb-1">RATE CONTROL</div>
                    <div
                        ref={joystickRef}
                        onMouseDown={handleJoystickDown}
                        className="w-32 h-32 bg-slate-900 rounded-full border-2 border-slate-600 relative cursor-crosshair select-none shadow-inner"
                    >
                        {/* Cross hairs */}
                        <div className="absolute top-1/2 w-full h-px bg-slate-700" />
                        <div className="absolute left-1/2 h-full w-px bg-slate-700" />

                        {/* Labels */}
                        <span className="absolute top-0.5 left-1/2 -translate-x-1/2 text-[8px] text-slate-600">▲ UP</span>
                        <span className="absolute bottom-0.5 left-1/2 -translate-x-1/2 text-[8px] text-slate-600">▼ DN</span>
                        <span className="absolute left-1 top-1/2 -translate-y-1/2 text-[8px] text-slate-600">◀</span>
                        <span className="absolute right-1 top-1/2 -translate-y-1/2 text-[8px] text-slate-600">▶</span>

                        {/* Stick knob */}
                        <div
                            className="w-6 h-6 rounded-full bg-gradient-to-br from-blue-400 to-blue-600 absolute shadow-lg border border-blue-300 transition-all duration-75"
                            style={{
                                left: `${50 + joystickPos.x * 40}%`,
                                top: `${50 + joystickPos.y * 40}%`,
                                transform: 'translate(-50%, -50%)'
                            }}
                        >
                            <div className="absolute inset-1 rounded-full bg-blue-300/30" />
                        </div>
                    </div>
                </div>

                {/* RIGHT: Quick Actions + Goto */}
                <div className="space-y-2">
                    {/* Goto Sliders */}
                    <div className="bg-slate-900/60 rounded-lg p-2">
                        <div className="text-[10px] text-slate-500 font-bold mb-1">GOTO ANGLE</div>
                        <div className="space-y-1">
                            <div className="flex items-center gap-1">
                                <span className="text-[9px] text-slate-500 w-5">P</span>
                                <input
                                    type="range" min={-90} max={30} value={targetPitch}
                                    onChange={(e) => setTargetPitch(Number(e.target.value))}
                                    className="flex-1 h-1 accent-emerald-500"
                                />
                                <span className="text-[10px] text-white font-mono w-8 text-right">{targetPitch}°</span>
                            </div>
                            <div className="flex items-center gap-1">
                                <span className="text-[9px] text-slate-500 w-5">Y</span>
                                <input
                                    type="range" min={-180} max={180} value={targetYaw}
                                    onChange={(e) => setTargetYaw(Number(e.target.value))}
                                    className="flex-1 h-1 accent-blue-500"
                                />
                                <span className="text-[10px] text-white font-mono w-8 text-right">{targetYaw}°</span>
                            </div>
                            <button onClick={sendGoto} className="w-full text-[10px] py-1 bg-indigo-600 text-white rounded font-bold hover:bg-indigo-500 transition-colors">
                                GOTO →
                            </button>
                        </div>
                    </div>

                    {/* Quick Actions */}
                    <div className="grid grid-cols-2 gap-1">
                        <button onClick={sendCenter} className="text-[10px] py-1.5 bg-slate-700 text-slate-200 rounded font-bold hover:bg-slate-600 transition-colors">
                            ⊕ CENTER
                        </button>
                        <button onClick={sendDown} className="text-[10px] py-1.5 bg-slate-700 text-slate-200 rounded font-bold hover:bg-slate-600 transition-colors">
                            ⤓ DOWN
                        </button>
                        <button onClick={sendPhoto} className="text-[10px] py-1.5 bg-sky-700 text-white rounded font-bold hover:bg-sky-600 transition-colors">
                            📷 PHOTO
                        </button>
                        <button
                            onClick={toggleRecord}
                            className={`text-[10px] py-1.5 rounded font-bold transition-colors ${isRecording ? 'bg-red-600 text-white animate-pulse' : 'bg-slate-700 text-slate-200 hover:bg-slate-600'}`}
                        >
                            {isRecording ? '⏹ STOP' : '🔴 REC'}
                        </button>
                    </div>
                </div>

            </div>
        </div>
    );
};

export default GimbalControl;

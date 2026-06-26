import React, { useState, useEffect } from 'react';

const VirtualRemote = ({ socket, activeDrone }) => {
    const [stickState, setStickState] = useState({
        throttle: 1000, yaw: 1500, pitch: 1500, roll: 1500
    });
    const [switches, setSwitches] = useState({
        swa: 1000, swb: 1000, swc: 1000, swd: 1000
    });

    const [isActive, setIsActive] = useState(false);

    // Keyboard Event Listener
    useEffect(() => {
        const handleKeyDown = (e) => {
            if (!isActive) return;
            const step = 50;

            setStickState(prev => {
                let next = { ...prev };
                switch (e.key.toLowerCase()) {
                    case 'w': next.throttle = Math.min(2000, prev.throttle + step); break;
                    case 's': next.throttle = Math.max(1000, prev.throttle - step); break;
                    case 'a': next.yaw = Math.max(1000, prev.yaw - step); break;
                    case 'd': next.yaw = Math.min(2000, prev.yaw + step); break;
                    case 'arrowup': next.pitch = Math.max(1000, prev.pitch - step); break; // Nose down
                    case 'arrowdown': next.pitch = Math.min(2000, prev.pitch + step); break; // Nose up
                    case 'arrowleft': next.roll = Math.max(1000, prev.roll - step); break;
                    case 'arrowright': next.roll = Math.min(2000, prev.roll + step); break;
                    case ' ': next.throttle = 1000; break; // Space = Kill Throttle (failsafe)
                    default: return prev;
                }
                return next;
            });
        };

        const handleKeyUp = (e) => {
            if (!isActive) return;
            setStickState(prev => {
                let next = { ...prev };
                switch (e.key.toLowerCase()) {
                    case 'a': case 'd': next.yaw = 1500; break;
                    case 'arrowup': case 'arrowdown': next.pitch = 1500; break;
                    case 'arrowleft': case 'arrowright': next.roll = 1500; break;
                    default: return prev;
                }
                return next;
            });
        };

        window.addEventListener('keydown', handleKeyDown);
        window.addEventListener('keyup', handleKeyUp);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
            window.removeEventListener('keyup', handleKeyUp);
        };
    }, [isActive]);

    // Send Control Loop (10Hz)
    useEffect(() => {
        if (!isActive) return;
        const interval = setInterval(() => {
            const channels = [
                stickState.roll, stickState.yaw, stickState.throttle, stickState.pitch,
                switches.swa, switches.swb, switches.swc, switches.swd
            ];
            socket.emit('control_input', { drone: activeDrone, channels });
        }, 100);
        return () => clearInterval(interval);
    }, [isActive, stickState, switches, activeDrone, socket]);

    const toggleSwitch = (sw) => {
        setSwitches(prev => ({
            ...prev,
            [sw]: prev[sw] === 1000 ? 2000 : 1000
        }));
    };

    return (
        <div className="flex flex-col items-center justify-center p-8 bg-slate-900 h-full select-none">

            {/* RADIO BODY */}
            <div className="bg-zinc-800 rounded-[3rem] p-8 shadow-2xl border-b-8 border-zinc-900 w-full max-w-4xl relative">

                {/* Antenna */}
                <div className="absolute -top-12 left-1/2 -translate-x-1/2 w-8 h-20 bg-zinc-900 rounded-t-full border-2 border-zinc-700"></div>

                {/* Handle */}
                <div className="absolute -top-16 left-1/2 -translate-x-1/2 w-48 h-16 border-t-8 border-x-8 border-zinc-400 rounded-t-3xl -z-10"></div>

                {/* TOP SWITCHES */}
                <div className="flex justify-between mb-8 px-8">
                    <div className="flex gap-4">
                        <div className="flex flex-col items-center">
                            <label className="text-zinc-400 text-[10px] font-bold mb-1">SwA (Arm)</label>
                            <button
                                onClick={() => toggleSwitch('swa')}
                                className={`w-4 h-12 rounded bg-zinc-600 border-2 ${switches.swa === 2000 ? 'border-green-500 bg-green-900' : 'border-zinc-500'} relative`}
                            >
                                <div className={`absolute w-full h-1/2 bg-zinc-300 rounded transition-all ${switches.swa === 2000 ? 'top-0' : 'bottom-0'}`}></div>
                            </button>
                        </div>
                        <div className="flex flex-col items-center">
                            <label className="text-zinc-400 text-[10px] font-bold mb-1">SwB (Mode)</label>
                            <button
                                onClick={() => toggleSwitch('swb')}
                                className={`w-4 h-12 rounded bg-zinc-600 border-2 ${switches.swb === 2000 ? 'border-orange-500 bg-orange-900' : 'border-zinc-500'} relative`}
                            >
                                <div className={`absolute w-full h-1/2 bg-zinc-300 rounded transition-all ${switches.swb === 2000 ? 'top-0' : 'bottom-0'}`}></div>
                            </button>
                        </div>
                    </div>

                    {/* Power Button */}
                    <button
                        onClick={() => setIsActive(!isActive)}
                        className={`w-16 h-16 rounded-full border-4 flex items-center justify-center shadow-inner transition-all ${isActive ? 'border-green-500 text-green-500 shadow-green-500/50' : 'border-red-900/50 text-zinc-600'}`}
                    >
                        <svg xmlns="http://www.w3.org/2000/svg" className="h-8 w-8" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                        </svg>
                    </button>

                    <div className="flex gap-4">
                        <div className="flex flex-col items-center">
                            <label className="text-zinc-400 text-[10px] font-bold mb-1">SwC</label>
                            <button
                                onClick={() => toggleSwitch('swc')}
                                className={`w-4 h-12 rounded bg-zinc-600 border-2 border-zinc-500 relative`}
                            >
                                <div className={`absolute w-full h-1/2 bg-zinc-300 rounded transition-all ${switches.swc === 2000 ? 'top-0' : 'bottom-0'}`}></div>
                            </button>
                        </div>
                        <div className="flex flex-col items-center">
                            <label className="text-zinc-400 text-[10px] font-bold mb-1">SwD</label>
                            <button
                                onClick={() => toggleSwitch('swd')}
                                className={`w-4 h-12 rounded bg-zinc-600 border-2 border-zinc-500 relative`}
                            >
                                <div className={`absolute w-full h-1/2 bg-zinc-300 rounded transition-all ${switches.swd === 2000 ? 'top-0' : 'bottom-0'}`}></div>
                            </button>
                        </div>
                    </div>
                </div>


                {/* MAIN GIMBALS AREA */}
                <div className="grid grid-cols-2 gap-12 px-8">

                    {/* LEFT GIMBAL (Mode 2: Throttle/Yaw) */}
                    <div className="flex flex-col items-center">
                        <div className="w-56 h-56 rounded-full bg-zinc-700 border-4 border-zinc-600 shadow-inner relative overflow-hidden group">
                            {/* Axis Lines */}
                            <div className="absolute top-1/2 w-full h-px bg-zinc-600/50"></div>
                            <div className="absolute left-1/2 h-full w-px bg-zinc-600/50"></div>

                            {/* Stick */}
                            <div
                                className="w-16 h-16 rounded-full bg-gradient-to-br from-zinc-200 to-zinc-400 absolute shadow-xl border border-zinc-400 z-10 transition-all duration-75"
                                style={{
                                    bottom: `${(stickState.throttle - 1000) / 10}%`,
                                    left: `${(stickState.yaw - 1000) / 10}%`,
                                    transform: 'translate(-50%, 50%)'
                                }}
                            >
                                <div className="absolute inset-0.5 border-2 border-zinc-300/50 rounded-full"></div>
                                <div className="absolute inset-0 flex items-center justify-center">
                                    <div className="w-1 h-1 bg-red-400 rounded-full"></div>
                                </div>
                            </div>
                        </div>
                        <span className="text-zinc-500 text-xs font-bold mt-2 tracking-widest">THROTTLE / YAW</span>
                    </div>

                    {/* RIGHT GIMBAL (Mode 2: Pitch/Roll) */}
                    <div className="flex flex-col items-center">
                        <div className="w-56 h-56 rounded-full bg-zinc-700 border-4 border-zinc-600 shadow-inner relative overflow-hidden group">
                            {/* Axis Lines */}
                            <div className="absolute top-1/2 w-full h-px bg-zinc-600/50"></div>
                            <div className="absolute left-1/2 h-full w-px bg-zinc-600/50"></div>

                            {/* Stick */}
                            <div
                                className="w-16 h-16 rounded-full bg-gradient-to-br from-zinc-200 to-zinc-400 absolute shadow-xl border border-zinc-400 z-10 transition-all duration-75"
                                style={{
                                    bottom: `${(stickState.pitch - 1000) / 10}%`,
                                    left: `${(stickState.roll - 1000) / 10}%`,
                                    transform: 'translate(-50%, 50%)'
                                }}
                            >
                                <div className="absolute inset-0.5 border-2 border-zinc-300/50 rounded-full"></div>
                                <div className="absolute inset-0 flex items-center justify-center">
                                    <div className="w-1 h-1 bg-blue-400 rounded-full"></div>
                                </div>
                            </div>
                        </div>
                        <span className="text-zinc-500 text-xs font-bold mt-2 tracking-widest">PITCH / ROLL</span>
                    </div>

                </div>

                {/* DIGITAL SCREEN */}
                <div className="mt-8 mx-auto w-64 h-32 bg-blue-100 rounded border-2 border-zinc-500 shadow-inner flex flex-col items-center justify-center p-2 font-mono text-[10px] text-blue-900 select-text">
                    <div className="flex w-full justify-between items-center border-b border-blue-300 pb-1 mb-1">
                        <span className="font-bold uppercase">{activeDrone}</span>
                        <span>{isActive ? 'TX: ON' : 'TX: OFF'}</span>
                        <span>BAT: 7.4V</span>
                    </div>
                    <div className="grid grid-cols-2 gap-x-8 gap-y-1 w-full flex-1 items-center">
                        <div className="flex justify-between"><span>THR</span> <span>{stickState.throttle - 1500}</span></div>
                        <div className="flex justify-between"><span>PIT</span> <span>{stickState.pitch - 1500}</span></div>
                        <div className="flex justify-between"><span>YAW</span> <span>{stickState.yaw - 1500}</span></div>
                        <div className="flex justify-between"><span>ROL</span> <span>{stickState.roll - 1500}</span></div>
                    </div>
                    <div className="w-full border-t border-blue-300 pt-1 mt-1 flex justify-between">
                        <span>SwA:{switches.swa > 1500 ? 'DN' : 'UP'}</span>
                        <span>SwB:{switches.swb > 1500 ? 'DN' : 'UP'}</span>
                    </div>
                </div>

                {/* SPEAKER GRILLS */}
                <div className="absolute bottom-6 left-6 flex gap-1">
                    <div className="w-1 h-8 bg-zinc-900 rounded-full"></div>
                    <div className="w-1 h-8 bg-zinc-900 rounded-full"></div>
                    <div className="w-1 h-8 bg-zinc-900 rounded-full"></div>
                </div>
                <div className="absolute bottom-6 right-6 flex gap-1">
                    <div className="w-1 h-8 bg-zinc-900 rounded-full"></div>
                    <div className="w-1 h-8 bg-zinc-900 rounded-full"></div>
                    <div className="w-1 h-8 bg-zinc-900 rounded-full"></div>
                </div>

            </div>

            <div className="mt-4 text-xs text-zinc-500 font-bold">
                JUMPER T12 EMULATION • MODE 2
            </div>
        </div>
    );
};

export default VirtualRemote; // explicit export



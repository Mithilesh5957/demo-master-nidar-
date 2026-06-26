import React from 'react';

const CameraView = ({ scoutIp }) => {
    return (
        <div className="flex flex-col h-full bg-slate-900 p-4 overflow-hidden">
            <h2 className="text-xl font-bold text-slate-100 mb-4 tracking-wide items-center justify-center flex gap-2">
                <span>🛰️ LIVE VISION FEED</span>
                <span className="text-xs font-normal text-slate-400 bg-slate-800 px-2 py-1 rounded">SCOUT ONE</span>
            </h2>

            <div className="flex-1 grid grid-cols-2 gap-4 min-h-0">
                {/* RGB MAIN STREAM */}
                <div className="flex flex-col bg-black rounded-xl overflow-hidden shadow-2xl border border-slate-700 relative group h-full">
                    <div className="absolute top-4 left-4 z-10">
                        <span className="px-2 py-1 bg-blue-600/90 text-white text-xs font-bold rounded shadow-sm backdrop-blur-sm animate-pulse">RGB CAMERA</span>
                    </div>
                    <img
                        src={`http://${scoutIp}:5001/video_feed`}
                        className="w-full h-full object-contain"
                        alt="RGB Feed"
                        onError={(e) => {
                            e.target.style.display = 'none';
                            e.target.parentNode.innerHTML += '<div class="absolute inset-0 flex items-center justify-center text-slate-500 font-mono">NO SIGNAL</div>';
                        }}
                    />
                    <div className="absolute bottom-0 inset-x-0 bg-gradient-to-t from-black/80 to-transparent p-4 translate-y-full group-hover:translate-y-0 transition-transform">
                        <p className="text-slate-200 text-xs font-mono">C12 RGB • AI TRACKING</p>
                    </div>
                </div>

                {/* THERMAL STREAM */}
                <div className="flex flex-col bg-black rounded-xl overflow-hidden shadow-2xl border border-slate-700 relative group h-full">
                    <div className="absolute top-4 left-4 z-10">
                        <span className="px-2 py-1 bg-orange-600/90 text-white text-xs font-bold rounded shadow-sm backdrop-blur-sm animate-pulse">THERMAL</span>
                    </div>
                    <img
                        src={`http://${scoutIp}:5001/thermal_feed`}
                        className="w-full h-full object-contain"
                        alt="Thermal Feed"
                        onError={(e) => {
                            e.target.style.display = 'none';
                            e.target.parentNode.innerHTML += '<div class="absolute inset-0 flex items-center justify-center text-slate-500 font-mono">NO SIGNAL</div>';
                        }}
                    />
                    <div className="absolute bottom-0 inset-x-0 bg-gradient-to-t from-black/80 to-transparent p-4 translate-y-full group-hover:translate-y-0 transition-transform">
                        <p className="text-slate-200 text-xs font-mono">C12 THERMAL • AI TRACKING</p>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default CameraView;

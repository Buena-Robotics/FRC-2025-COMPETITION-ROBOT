import { Akit } from "./lib/akit";

Akit.clean_custom_assets();
Akit.publish_field2d("robotics_room.png", {
    name: "RoboticsRoom",
    topLeft: [30,100],
    bottomRight: [1000,1000],
    widthInches: 323.04, // Real width of the field (long side)
    heightInches: 340.683, // Real height of the field (short side)
    defaultOrigin: "blue"
})
Akit.publish_field3d("robotics_room_3d.glb", {
    name: "RoboticsRoom3d", // Unique name, required for all asset types
    rotations: [],
    widthInches: 336, // Real width of the field (long side)
    heightInches: 336, // Real height of the field (short side)
    defaultOrigin: "blue", // Default origin location, "auto" if unspecified
    driverStations: [],
    gamePieces: [
        // {
        //     name: "coral",
        //     rotations: [],
        //     position: [0,0,0],
        //     stagedObjects: []
        // },
        // {
        //     name: "algae",
        //     rotations: [],
        //     position: [0,0,0],
        //     stagedObjects: []
        // }
    ]
});
Akit.publish_robot("nerds_2025_robot.glb", {
    name: "Nerds2025",
    rotations: [{"axis":"z","degrees":-90}],
    position: [-0.3,0.3,0], // Position offset in meters, applied after rotation
    cameras: [],
    components: [] // See "Articulated Components"
});
import { Akit } from "./lib/akit";

Akit.clean_custom_assets();
Akit.publish_field2d("robotics_room.png", {
    name: "robotics_room",
    topLeft: [0, 200],
    bottomRight: [200, 0],
    widthInches: 313, // Real width of the field (long side)
    heightInches: 313, // Real height of the field (short side)
    defaultOrigin: "blue"
})
Akit.publish_field3d("robotics_room.glb", {
    name: "robotics_room_3d", // Unique name, required for all asset types
    rotations: [],
    widthInches: 313, // Real width of the field (long side)
    heightInches: 313, // Real height of the field (short side)
    defaultOrigin: "blue", // Default origin location, "auto" if unspecified
    driverStations: [],
    gamePieces: [
        {
            name: "coral",
            rotations: [],
            position: [0,0,0],
            stagedObjects: []
        },
        {
            name: "algae",
            rotations: [],
            position: [0,0,0],
            stagedObjects: []
        }
    ]
});
// Akit.publish_robot("NERDS_2025_ROBOT.glb", {
//     name: "NERDS_2025_ROBOT",
//     rotations: [],
//     position: [0,0,0], // Position offset in meters, applied after rotation
//     cameras: [// Fixed camera positions, can be empty
//     {
//         name: "", // Camera name
//         rotations: [],
//         position: [0, 0, 0], // Position offset in meters relative to the robot, applied after rotation
//         resolution: [0, 0], // Resolution in pixels, used to set the fixed aspect ratio
//         fov: 0 // Horizontal field of view in degrees
//     }],
//     components: [
    
//     ] // See "Articulated Components"
// });
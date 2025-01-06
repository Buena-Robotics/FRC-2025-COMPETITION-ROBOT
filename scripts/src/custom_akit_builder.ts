import { Akit } from "./lib/akit";

Akit.clean_custom_assets();
Akit.publish_robot("NERDS_2025_ROBOT.glb", {
    name: "NERDS_2025_ROBOT",
    rotations: [], // Sequence of rotations along the x, y, and z axes
    position: [0,0,0], // Position offset in meters, applied after rotation
    cameras: [// Fixed camera positions, can be empty
    {
        name: "", // Camera name
        rotations: [], // Sequence of rotations along the x, y, and z axes
        position: [0, 0, 0], // Position offset in meters relative to the robot, applied after rotation
        resolution: [0, 0], // Resolution in pixels, used to set the fixed aspect ratio
        fov: 0 // Horizontal field of view in degrees
    }],
    components: [
    
    ] // See "Articulated Components"
});
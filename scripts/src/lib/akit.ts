import * as fs from 'fs';

// https://docs.advantagescope.org/more-features/custom-assets/
export namespace Akit {
    const assets_folder_path = 'assets/';
    const akit_custom_assets_folder_path = 'C:\\Users\\raygo\\AppData\\Roaming\\AdvantageScope\\userAssets\\';

    type TypeName = "Field2d"|"Field3d"|"Robot"|"Joystick";
    type FolderName = `${TypeName}_${string}`;

    export interface Rotation {
        axis: "x" | "y" | "z";
        degrees: number;
    }
    export type Position3D = [number, number, number];

    // A model must be included in the folder with the name "model.glb". CAD files must be converted to glTF; see this page for details. The config file must be in the following format:
    export interface Robot {
        name: string; // Unique name, required for all asset types
        sourceUrl?: string; // Link to the original file, optional
        disableSimplification?: boolean; // Whether to disable model simplification, optional
        rotations: Rotation[]; // Sequence of rotations along the x, y, and z axes
        position: Position3D; // Position offset in meters, applied after rotation
        cameras: // Fixed camera positions, can be empty
        {
            name: string // Camera name
            rotations: Rotation[] // Sequence of rotations along the x, y, and z axes
            position: Position3D // Position offset in meters relative to the robot, applied after rotation
            resolution: [number, number] // Resolution in pixels, used to set the fixed aspect ratio
            fov: number // Horizontal field of view in degrees
        }[];
        components: ArticulatedComponents[]; // See "Articulated Components"
    }

    export interface ArticulatedComponents {
        zeroedRotations: Rotation[]; // Sequence of rotations along the x, y, and z axes
        zeroedPosition: Position3D; // Position offset in meters relative to the robot, applied after rotation
    }

    // An image must be included in the folder with the name "image.png". The config file must be in the following format:
    export interface Joystick {
        name: string; // Unique name, required for all asset types
        components: (Joystick__SingleButton_POVValue|Joystick__TwoAxis|Joystick__SingleAxis)[]; // Array of component configurations, see below
    }
    export interface Joystick__SingleButton_POVValue {
        type: "button";
        isYellow: boolean;
        isEllipse: boolean;
        centerPx: [number, number];
        sizePx: [number, number];
        sourceIndex: number;
        sourcePov?: "up"|"right"|"down"|"left"; //If provided, the "sourceIndex" will be the index of the POV to read.
    }
    export interface Joystick__TwoAxis {
        type: "joystick"; // A joystick that moves in two dimensions
        isYellow: boolean;
        centerPx: [number, number];
        radiusPx: number;
        xSourceIndex: number;
        xSourceInverted: boolean; // Not inverted: right = positive
        ySourceIndex: number;
        ySourceInverted: boolean; // Not inverted: up = positive
        buttonSourceIndex?: number; // Optional
    }
    export interface Joystick__SingleAxis {
        type: "axis"; // A single axis value
        isYellow: boolean;
        centerPx: [number, number];
        sizePx: [number, number];
        sourceIndex: number;
        sourceRange: [number, number]; // Min greater than max to invert
    }

    // An image must be included in the folder with the name "image.png". It should be oriented with the blue alliance on the left. The config file must be in the following format:
    export interface FieldFlat {
        name: string; // Unique name, required for all asset types
        sourceUrl?: string; // Link to the original file, optional
        topLeft: [number, number]; // Pixel coordinate (origin at upper left)
        bottomRight: [number, number]; // Pixel coordinate (origin at upper left)
        widthInches: number; // Real width of the field (long side)
        heightInches: number; // Real height of the field (short side)
        defaultOrigin: "auto" | "blue" | "red"; // Default origin location, "auto" if unspecified
    }

    // A model must be included in the folder with the name "model.glb". After all rotations are applied, the field should be oriented with the blue alliance on the left. CAD files must be converted to glTF; see this page for details. Game piece models follow the naming convention "model_INDEX.glb" based on the order that they appear in the "gamePieces" array.
    export interface Field3d {
        name: string; // Unique name, required for all asset types
        sourceUrl?: string; // Link to the original file, optional
        rotations: Rotation[]; // Sequence of rotations along the x, y, and z axes
        widthInches: number; // Real width of the field (long side)
        heightInches: number; // Real height of the field (short side)
        defaultOrigin: "auto" | "blue" | "red"; // Default origin location, "auto" if unspecified
        driverStations: [
            [number, number][] // Driver station positions (X & Y in meters relative to the center of the field), ordered B1, B2, B3, R1, R2, R3
        ]
        gamePieces: [ // List of game piece types
            {
                name: string; // Game piece name
                rotations: Rotation[]; // Sequence of rotations along the x, y, and z axes
                position: Position3D; // Position offset in meters, applied after rotation
                stagedObjects: string[]; // Names of staged game piece objects, to hide if user poses are supplied
            }
        ]
    }

    function is_custom_asset_folder_name(file: string){
        return file.includes("Field2d") || 
            file.includes("Field3d") || 
            file.includes("Robot") || 
            file.includes("Joystick");
    }
    // Run before publishing custom_assets
    export function clean_custom_assets(){
        for(const file of fs.readdirSync(akit_custom_assets_folder_path)){
            if(fs.statSync(akit_custom_assets_folder_path + file, {throwIfNoEntry: false})?.isDirectory && is_custom_asset_folder_name(file))
                fs.rmSync(akit_custom_assets_folder_path + file, {force: true});
        }
    }
    function folder_name(type: TypeName, name: string): FolderName{ return `${type}_${name}`; }
    function create_folder(folder_name: FolderName){
        const full_path = akit_custom_assets_folder_path + folder_name;
        fs.mkdirSync(full_path); 
        return full_path; 
    }

    function publish_misc(type: TypeName, name: string, named_obj: {name: string}){
        const folder_path = create_folder(folder_name(type, named_obj.name));
        fs.cpSync(assets_folder_path + name, folder_path + '/image.png');
        fs.writeFileSync(folder_path + '/config.json', JSON.stringify(named_obj), 'utf-8');
    }

    export function publish_field2d(image_name: string, field2d: FieldFlat){
        publish_misc('Field2d', image_name, field2d);
    }
    export function publish_joystick(image_name: string, joystick: Joystick){
        publish_misc('Joystick', image_name, joystick);
    }
    export function publish_field3d(model_name: string, field3d: Field3d){
        publish_misc('Field3d', model_name, field3d);
    }
    export function publish_robot(model_name: string, robot: Robot){
        publish_misc('Robot', model_name, robot);
    }
}
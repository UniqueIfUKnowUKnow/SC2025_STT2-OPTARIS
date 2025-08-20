declare module 'three/examples/jsm/controls/OrbitControls' {
    export class OrbitControls {
        constructor(object: any, domElement?: any);
        update(): void;
        dispose(): void;
        enableDamping: boolean;
        dampingFactor: number;
        screenSpacePanning: boolean;
        minDistance: number;
        maxDistance: number;
    }
}



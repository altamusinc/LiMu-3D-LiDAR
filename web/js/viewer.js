import * as THREE from "https://esm.sh/three";
import { OrbitControls } from "https://esm.sh/three/examples/jsm/controls/OrbitControls.js";

const scene = new THREE.Scene();
scene.background = new THREE.Color( 0xffffff );
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ alpha: true });

let frame_time = 111; // in milliseconds

window.scene = scene; //send scene to global scope
window.frame_time = frame_time; //send frame_time to global scope

renderer.setSize($('.viewer').innerWidth(), $('.viewer').innerHeight());
$('.viewer').html(renderer.domElement);

const controls = new OrbitControls( camera, renderer.domElement );
const fileloader = new THREE.FileLoader()

function getLatestFile(){

    async function success(data) {

        if(data==null){
            console.log(data);
            await new Promise(r => setTimeout(r, 2000));
            getLatestFile();
            return true;
        }

        try {
            let points = [];
            let colors = [];
    
            let xyz_size = 12; // in bytes
            let color_size = 3; // in bytes
            let point_size = xyz_size + color_size; // in bytes
            let num_points = data.byteLength / point_size
    
            let pointsView = new DataView(data, 0, num_points * xyz_size)
            let colorsView = new DataView(data, num_points * xyz_size)

            for (let i = 0; i < num_points; i += 1) {
                let x = pointsView.getFloat32((i * xyz_size) + 0, true);
                let y = pointsView.getFloat32((i * xyz_size) + 4, true);
                let z = pointsView.getFloat32((i * xyz_size) + 8, true);
                points.push(x, y, z);
            }

            for (let i = 0; i < num_points; i += 1) {
                let r = colorsView.getUint8((i * color_size) + 0, true) / 255;
                let g = colorsView.getUint8((i * color_size) + 1, true) / 255;
                let b = colorsView.getUint8((i * color_size) + 2, true) / 255;
                colors.push(r, g, b);
            }

            let old_pointclolud = scene.getObjectByName('pointcloud');
            let size = 0.01;
            if(old_pointclolud){size = old_pointclolud.material.size;}

            let geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(points), 3));
            geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));
            const material = new THREE.PointsMaterial({ size: size, vertexColors: true });
            let pointcloud = new THREE.Points(geometry, material);
            pointcloud.name = 'pointcloud';
            scene.remove(old_pointclolud);
            scene.add(pointcloud);

            frame_time = window.frame_time;

            await new Promise(r => setTimeout(r, frame_time));
            getLatestFile();

        } 
        catch (error) {
            console.log(error)
            await new Promise(r => setTimeout(r, 2000));
            getLatestFile();
        }
        
    }
    
    async function error(error){
        console.log(error)
        await new Promise(r => setTimeout(r, 2000));
        getLatestFile();
    }

    
    $.ajax({url: '/asbytes', method: "GET", xhrFields: {responseType: "arraybuffer"}, success, error});
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

camera.position.set(2, 2, 2);
renderer.render(scene, camera);
animate();
getLatestFile()

import * as THREE from "https://esm.sh/three";
import { OrbitControls } from "https://esm.sh/three/examples/jsm/controls/OrbitControls.js";

const scene = new THREE.Scene();
scene.background = new THREE.Color( 0xffffff );
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ alpha: true });

renderer.setSize($('#content').innerWidth(), $('#content').innerHeight());
$('#content').html(renderer.domElement);

const controls = new OrbitControls( camera, renderer.domElement );
const fileloader = new THREE.FileLoader()

function getLatestFile(){

    async function success(data) {

        let points = [];
        let colors = [];

        let xyz_size = 12; // in bytes
        let color_size = 3; // in bytes
        let point_size = xyz_size + color_size; // in bytes
        let num_points = data.byteLength / point_size

        let pointsView = new DataView(data, 0, num_points * xyz_size)
        let colorsView = new DataView(data, num_points * xyz_size)

        try {
            for (let i = 0; i < num_points; i += 1) {
                let x = pointsView.getFloat32((i * xyz_size) + 0, true);
                let y = pointsView.getFloat32((i * xyz_size) + 4, true);
                let z = pointsView.getFloat32((i * xyz_size) + 8, true);
                points.push([x, y, z]);
            }

            for (let i = 0; i < num_points; i += 1) {
                let r = colorsView.getUint8((i * color_size) + 0, true) / 255;
                let g = colorsView.getUint8((i * color_size) + 1, true) / 255;
                let b = colorsView.getUint8((i * color_size) + 2, true) / 255;
                colors.push([r, g, b]);
            }

            points = points.flat();
            colors = colors.flat();

            let geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(points), 3));
            geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));
            const material = new THREE.PointsMaterial({ size: 0.01, vertexColors: true });
            let pointcloud = new THREE.Points(geometry, material);
            pointcloud.name = 'pointcloud';
            // pointcloud.material.color.set(new THREE.Color(0x888888));
            let old_pointclolud = scene.getObjectByName('pointcloud');
            scene.remove(old_pointclolud);
            scene.add(pointcloud);

        } catch (error) {
            console.log(error)
        }


        // await new Promise(r => setTimeout(r, 2000));
        getLatestFile();
    }
    
    async function error(error){
        // await new Promise(r => setTimeout(r, 2000));
        // getLatestFile();
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

function turnOn(){
    $.post("/api/on");
}

function turnOff(){
    $.post("/api/off");
}

function color_webcam(){
    $.post("/api/color_webcam");
}

function color_default(){
    $.post("/api/color_default");
}

$('#on_button').on('click', turnOn);
$('#off_button').on('click', turnOff);
$('#webcam_color_button').on('click', color_webcam);
$('#default_color_button').on('click', color_default);
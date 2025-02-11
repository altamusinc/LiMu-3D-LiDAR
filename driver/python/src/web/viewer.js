import * as THREE from "https://esm.sh/three";
import { PCDLoader } from 'https://esm.sh/three/addons/loaders/PCDLoader.js';
import { OrbitControls } from "https://esm.sh/three/examples/jsm/controls/OrbitControls.js";
import { XYZLoader } from 'https://esm.sh/three/addons/loaders/XYZLoader.js';
const xyzloader = new XYZLoader();

const scene = new THREE.Scene();
scene.background = new THREE.Color( 0xffffff );
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ alpha: true });

renderer.setSize($('#content').innerWidth(), $('#content').innerHeight());
$('#content').html(renderer.domElement);

const controls = new OrbitControls( camera, renderer.domElement );
const loader = new PCDLoader();

function getLatestFile() {
    async function success(object3D){

        const material = new THREE.PointsMaterial( { size: 0.01 } );
        let points = new THREE.Points( object3D, material );
        points.name = 'pointcloud';
        points.material.color.set(new THREE.Color(0x000000));  
        let old_pointclolud = scene.getObjectByName('pointcloud');
        scene.remove(old_pointclolud);
        scene.add(points);
        // await new Promise(r => setTimeout(r, 2000));
        getLatestFile();
    }
    
    async function error(error){
        await new Promise(r => setTimeout(r, 2000));
        getLatestFile();
    }
    
    async function progress(progress){
        // console.log(progress)
    }
    
    xyzloader.load('/points', success, progress, error);
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
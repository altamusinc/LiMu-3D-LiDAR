import * as THREE from "https://esm.sh/three";
import { PCDLoader } from 'https://esm.sh/three/addons/loaders/PCDLoader.js';
import { OrbitControls } from "https://esm.sh/three/examples/jsm/controls/OrbitControls.js";

const scene = new THREE.Scene();
scene.background = new THREE.Color( 0xffffff );
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ alpha: true });

renderer.setSize($('#content').innerWidth(), $('#content').innerHeight());
$('#content').html(renderer.domElement);

const controls = new OrbitControls( camera, renderer.domElement );
const loader = new PCDLoader();

function getLatestFile() {
    function success(object3D){
        console.log(object3D)
        object3D.name = 'pointcloud';  
        let old_pointclolud = scene.getObjectByName('pointcloud');
        scene.remove(old_pointclolud);
        scene.add(object3D);
        getLatestFile();
    }

    function error(error){
        // console.log(error)
    }

    function progress(progress){
        // console.log(progress)
    }

    loader.load('/points', success, progress, error);
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

function foo(){
    console.log("clicked")
}

$('#this_button').on('click', foo);
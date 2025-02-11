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

// import { PCDLoader } from 'https://esm.sh/three/addons/loaders/PCDLoader.js';
// const pcdloader = new PCDLoader();

// import { XYZLoader } from 'https://esm.sh/three/addons/loaders/XYZLoader.js';
// const xyzloader = new XYZLoader();

function getLatestFile(){

    async function success(data){     
         
        let dataview = new DataView(data);
        let buffer_length = dataview.buffer.byteLength;

		let geometry = new THREE.BufferGeometry();

        let points = [];
        for (let i=0; i<buffer_length/2; i+=24){
            let x = dataview.getFloat64(i,true);
            let y = dataview.getFloat64(i+8,true);
            let z = dataview.getFloat64(i+16,true);
            points.push([x,y,z]);
        }
        points = points.flat();
		geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(points),3));

        // let colors = [];
        // for (let i=buffer_length/2; i<buffer_length; i+=24){
        //     let r = dataview.getFloat64(i,true);
        //     let g = dataview.getFloat64(i+8,true);
        //     let b = dataview.getFloat64(i+16,true);
        //     colors.push([r,g,b]);
        // }
        // colors = colors.flat();
		// geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors),3));

        const material = new THREE.PointsMaterial( { size: 0.01, vertexColors: true } );
        let pointcloud = new THREE.Points( geometry, material );
        pointcloud.name = 'pointcloud';
        pointcloud.material.color.set(new THREE.Color(0x111111)); pointcloud.material.vertexColors = false;
        let old_pointclolud = scene.getObjectByName('pointcloud');
        scene.remove(old_pointclolud);
        scene.add(pointcloud);
        
        // await new Promise(r => setTimeout(r, 2000));
        getLatestFile();
    }
    
    async function error(error){
        // await new Promise(r => setTimeout(r, 2000));
        // getLatestFile();
    }

    
    $.ajax({url: 'http://wdueease-puget:5000/asbytes', method: "GET", xhrFields: {responseType: "arraybuffer"}, success, error});
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
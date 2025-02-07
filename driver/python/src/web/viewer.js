import * as THREE from "https://esm.sh/three";
import { OrbitControls } from "https://esm.sh/three/examples/jsm/controls/OrbitControls.js";

const scene = new THREE.Scene();
scene.background = new THREE.Color( 0xffffff );
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ alpha: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);
const controls = new OrbitControls( camera, renderer.domElement );

function processPointCloud(data){
    console.log("process start");
    const geometry = new THREE.BufferGeometry();
    const material = new THREE.PointsMaterial({ color: 0x000000, size: 0.01 });
    const vertices = new Float32Array(data.length * 3);            
    for (let i = 0; i < data.length; i++) {
        vertices[i * 3] = data[i][0];
        vertices[i * 3 + 1] = data[i][1];
        vertices[i * 3 + 2] = data[i][2];
    }            
    geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));    
    const points = new THREE.Points(geometry, material);
    points.name = 'pointcloud';  
    console.log("process stop");  
    return points;
}

function initialize(){
    $.ajax({url:'http://127.0.0.1:5000/points', datatype: 'json', success:function(data){
            console.log("Init start");
            let new_pointcloud = processPointCloud(data);
            scene.add(new_pointcloud);
            camera.position.set( 2, 2, 2 );
            renderer.render(scene, camera);
            animate();
        console.log("Init stop");
    },
        error: function (data) {
            console.log("error");
            console.log(data);
        }
    });
    function animate() {
        requestAnimationFrame(animate);
        controls.update();    
        renderer.render( scene, camera );    
    }

}

function update(){
    console.log("enter update");
    $.ajax({url:'http://127.0.0.1:5000/points', datatype: 'json', success:function(data){
            console.log("hello")
            console.log(data)
            let old_pointclolud = scene.getObjectByName('pointcloud');
            let new_pointcloud = processPointCloud(data);
            scene.remove(old_pointclolud);
            scene.add(new_pointcloud);
            renderer.render( scene, camera ); 
        }, 
    });
    console.log("exit update");
}

initialize();

const intervalId = setInterval(update, 5000);

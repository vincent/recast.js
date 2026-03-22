/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
'use strict';

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';
import { ConvexGeometry } from 'three/addons/geometries/ConvexGeometry.js';
import Stats from 'three/addons/libs/stats.module.js';

(async () => {
const recast = await Recast();

var renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setSize(document.body.clientWidth * 0.6, document.body.clientHeight * 0.8);
renderer.domElement.style.position = 'absolute';
renderer.domElement.style.bottom = 0;
renderer.domElement.style.right = 0;
document.body.appendChild(renderer.domElement);
renderer.setClearColor(0xffffff, 1.0);
renderer.clear();

var width = renderer.domElement.width;
var height = renderer.domElement.height;
var camera = new THREE.PerspectiveCamera( 45, width / height, 1, 10000);
camera.position.y = 50;
camera.position.z = 50;

var controls = new OrbitControls(camera, renderer.domElement);
controls.addEventListener('change', function(){
    render();
});

var scene = new THREE.Scene();
window.scene = scene;
window.navigationMesh = null;

var agent = new THREE.Object3D();
var agentBody = new THREE.Mesh(
    new THREE.CylinderGeometry(0.2, 0.5, 2),
    new THREE.MeshBasicMaterial({
      color: '#FF0000'
    })
);
agentBody.position.y = 1;
agent.add(agentBody);

agent.arrowHelper = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 1),
    new THREE.Vector3(0, 0, 0),
    1.5,
    0xffff00);
agent.add(agent.arrowHelper);

scene.add(agent);

const light = new THREE.PointLight(0xffffff, 200.0)
light.position.y = 20;
light.position.x = 20;
scene.add(light);

////////////////////////////////

var terrain, navigationMesh, doors = {};

function render () {
    renderer.render(scene, camera);
}

function addMeshFromVertices (vertices, parent, plain) {
    var material = new THREE.MeshBasicMaterial({
        color: 0xFF0000,
        flatShading: true,
        side: THREE.DoubleSide,
        wireframe: ! plain,
        transparent: true,
        opacity: 0.3
    });

    for (var i = 0; i < vertices.length; i++) {
        if (!vertices[i+2]) { break; }

        var geometry = new ConvexGeometry([
            new THREE.Vector3(   vertices[i].x,   vertices[i].y,   vertices[i].z ),
            new THREE.Vector3( vertices[i+1].x, vertices[i+1].y, vertices[i+1].z ),
            new THREE.Vector3( vertices[i+2].x, vertices[i+2].y, vertices[i+2].z )
        ]);

        parent.add(new THREE.Mesh(geometry, material));

        i += 2;
    }
}

recast.settings({
    cellSize: 0.1,
    cellHeight: 0.05,
    agentHeight: 1.8,
    agentRadius: 0.4,
    agentMaxClimb: 2.0,
    agentMaxSlope: 30.0
});


var stats = new Stats();
stats.dom.style.position = 'absolute';
stats.dom.style.right = '0px';
stats.dom.style.bottom = '0px';
// document.body.appendChild( stats.dom );

var objLoader = new OBJLoader();
objLoader.load('doors.obj', async function(object){
    terrain = object;

    const mtlLoader = new MTLLoader();
    const materials = await mtlLoader.loadAsync('doors.mtl');
    objLoader.setMaterials( materials );
    object.traverse(function(child) {
        if (child instanceof THREE.Mesh) {
            child.geometry.computeVertexNormals();
            child.material.side = THREE.DoubleSide;
        }
    });
    scene.add(object);

    /**
     * Load an .OBJ file
     */
    recast.OBJLoader('doors.obj', function(){

        /**
         * Get navmesh geometry and draw it
         */
        if (location.search.match(/navigationmesh=1/)) {
            recast.getNavMeshVertices(recast.cb(function (vertices) {
                navigationMesh = new THREE.Object3D();
                window.navigationMesh = navigationMesh;
                addMeshFromVertices(vertices, navigationMesh);
            }));
        }

        /**
         * Load zones
         */
        recast.setZones({
            door1: { name: 'door1', refs: [ 69  ], flags: [ 16 ] },
            door2: { name: 'door2', refs: [ 103 ], flags: [ 16 ] },
            door3: { name: 'door3', refs: [ 84  ], flags: [ 16 ] }
        });

        recast.vent.on('update', function (agents) {
            if (agents[0]) agent.position.set(
                agents[0].position.x,
                agents[0].position.y,
                agents[0].position.z
            );
        });

        var hop = 0, hops = [
            {
                x: 87.5,
                y: 0,
                z: 10.8
            },
            {
                x: 1.6,
                y: 0,
                z: -12.9
            },
            {
                x: 43.9,
                y: 0,
                z: 9.6
            },
            {
                x: 80.1,
                y: 0,
                z: -11
            }
        ];

        recast.addAgent({
            position: hops[3],
            radius: 0.8,
            height: 0.5,
            maxAcceleration: 1.0,
            maxSpeed: 2.0,
            updateFlags: 0,
            separationWeight: 20.0
        });

        var last = new Date().getTime();
        var animate = function animate (time) {
            window.requestAnimationFrame(animate);

            if (window.TWEEN) window.TWEEN.update();
            recast.crowdUpdate(0.1);
            recast.crowdGetActiveAgents();

            last = time;
            render();

            stats.update();
        };

        var move = function (index) {
            var hop = index !== undefined ? index : hops[~~(Math.random() * hops.length)];
            recast.crowdRequestMoveTarget(0, hop.x, hop.y, hop.z);
        };

        window.toggleDoor = function (doorName) {
            recast.zones[doorName].toggleFlags(recast.FLAG_DISABLED);
        };

        move();
        setInterval(move, 5 * 1000);

        animate(new Date().getTime());
    });
})

})();

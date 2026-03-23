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
var camera = new THREE.PerspectiveCamera( 45, width / height, 1, 100000);
camera.position.y = 50;
camera.position.z = 50;

var controls = new OrbitControls(camera, renderer.domElement);
controls.addEventListener('change', function(){
    render();
});

var agentsParam = location.search.match(/agents=(\d+)/);
var MAX_AGENTS = (agentsParam && agentsParam.length == 2 ) ? agentsParam[1] : 10;
var MAX_HOPS = 10;

var agentsObjects = [];
var scene = new THREE.Scene();
window.scene = scene;
window.navigationMesh = null;

var agentGeometry = new THREE.CylinderGeometry(0.02, 0.05, 0.2);

for (var i = 0; i < MAX_AGENTS; i++) {
    var agent = new THREE.Object3D();
    var agentBody = new THREE.Mesh(
        agentGeometry,
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

    agentsObjects.push(agent);
    scene.add(agent);
}

var light = new THREE.SpotLight(0xffffff, 100);
light.position.set( 170, 330, -160 );
scene.add(light);
scene.add(new THREE.AmbientLight(0xffffff, 1.0));

var navigationMesh;

var createSpline = function(start, end, elevation) {
    const points = [
        start,
        new THREE.Vector3(
            (start.x + end.x)  / 2,
            ((start.y + end.y) / 2) + elevation,
            (start.z + end.z)  / 2
        ),
        end
    ];

    var geometry = new THREE.BufferGeometry().setFromPoints(points);
    var material = new THREE.LineBasicMaterial({
      color: 0x00ff00,
      transparent: true,
      opacity: 0.6,
      vertexColors: true
    });

    return new THREE.Line(geometry, material);
};

////////////////////////////////

var terrain, agents = [];

function render () {
    renderer.render(scene, camera);
}

var stats = new Stats();
stats.dom.style.position = 'absolute';
stats.dom.style.right = '0px';
stats.dom.style.bottom = '0px';
document.body.appendChild( stats.dom );

// Load visual mesh with OBJ + MTL
const objLoader = new OBJLoader();
const mtlLoader = new MTLLoader();
try {
    const materials = await mtlLoader.loadAsync('Paris/Paris2010.mtl');
    materials.preload();
    objLoader.setMaterials(materials);
} catch (e) {
    // MTL not required
}
objLoader.load('Paris/Paris2010_0.obj', function(object){
    terrain = object;
    object.traverse(function(child) {
        if (child instanceof THREE.Mesh) {
            child.geometry.computeVertexNormals();
            child.material.side = THREE.DoubleSide;
        }
    } );
    scene.add(object);
});

render();

/**
 * Load an .OBJ file
 */
recast.OBJLoader('Paris/Paris2010_0.obj', function(){

    recast.build();
    recast.initCrowd(1000, 1.0);

    /**
     * Get navmesh geometry and draw it
     */
    if (location.search.match(/navigationmesh=1/)) {
        recast.getNavMeshVertices(recast.cb(function (vertices) {

            navigationMesh = new THREE.Object3D();
            window.navigationMesh = navigationMesh;

            var material = new THREE.MeshBasicMaterial({
                color: 0x000055,
                flatShading: true,
                side: THREE.DoubleSide,
                wireframe: false,
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

                navigationMesh.add(new THREE.Mesh(geometry, material));

                i += 2;
            }
        }));
    }

    recast.vent.on('update', function (agentsList) {
        for (var i = 0; i < agentsList.length; i++) {
            var ag = agentsList[i];

            var angle = Math.atan2(- ag.velocity.z, ag.velocity.x);
            if (Math.abs(agentsObjects[ag.idx].rotation.y - angle) > 0) {
                agentsObjects[ag.idx].rotation.y = angle;
            }

            agentsObjects[ag.idx].position.set(
                ag.position.x,
                ag.position.y,
                ag.position.z
            );

            var color = ag.neighbors * 128;
            agentsObjects[ag.idx].children[0].material.color.set(color, color, color);
        }
    });

    /**
     * Add some agents
     */
    for (var i = 0; i < agentsObjects.length; i++) {
        agents.push(recast.addAgent({
            position: {
                x: 0,
                y: 0,
                z: -2.20
            },
            radius: 0.004,
            height: 0.5,
            maxAcceleration: 1.0,
            maxSpeed: 2.0,
            updateFlags: 0,
            separationWeight: 20.0
        }));
    }

    var routes;

    var animate = function animate (time) {
        window.requestAnimationFrame(animate);

        recast.crowdUpdate(0.1);
        recast.crowdGetActiveAgents();

        render();

        stats.update();
    };

    animate(new Date().getTime());

    window.sequence = function() {
        document.getElementById('sequence').style.display = 'none';
        routes = 0;
        goAway();
    };

    var goAway = function(){
        for (var i = 0; i < agentsObjects.length; i++) {
            (function (agentIdx) {
                recast.getRandomPoint(recast.cb(function(pt2x, pt2y, pt2z){
                    recast.crowdRequestMoveTarget(agentIdx, pt2x, pt2y, pt2z);
                    if (++routes < MAX_HOPS) {
                        setTimeout(goAway, 8000 * Math.random());
                    } else {
                        document.getElementById('sequence').style.display = 'block';
                    }
                }));
            })(i);
        }
    };

    window.sequence();
});

})();

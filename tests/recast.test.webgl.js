/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
/*jshint onevar: false, indent:4 */
/*global exports: true, require: true, THREE: true, Stats: true, $: true */
'use strict';

var renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setSize($(document.body).width(), $(document.body).width() * 3/4);
// renderer.domElement.style.position = 'absolute';
// renderer.domElement.style.bottom = 0;
// renderer.domElement.style.right = 0;
document.getElementById('wrapper').appendChild(renderer.domElement);
renderer.setClearColor(0xFFFFFF, 1.0);
renderer.clear();

var mouse = new THREE.Vector2();
var raycaster = new THREE.Raycaster();
renderer.domElement.addEventListener('click', function (e) {
    raycast(e, function (point) {
        recast.findNearestPoint(point.x, point.y, point.z, 2, 2, 2,
            recast.cb(function(x, y, z){

                if (e.shiftKey) {
                    addOffMeshLink(x, y, z);

                } else {
                    addObstacle(x, y, z);
                }
            }));
    });

});

var width = renderer.domElement.width;
var height = renderer.domElement.height;
var camera = new THREE.PerspectiveCamera( 45, width / height, 1, 10000);
camera.position.y = 70;
camera.position.z = 100;
camera.lookAt(new THREE.Vector3());

var controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.addEventListener('change', function(){
    render();
});

var agentsParam = location.search.match(/agents=(\d+)/);
var MAX_AGENTS = (agentsParam && agentsParam.length == 2 ) ? agentsParam[1] : 10;
var MAX_HOPS = 10;

var agentsObjects = [];
var scene = new THREE.Scene();

var agentGeometry = new THREE.CylinderGeometry(0.2, 0.5, 2);

for (var i = 0; i < MAX_AGENTS; i++) {
    var agent = new THREE.Object3D();
    var agentBody = new THREE.Mesh(
        agentGeometry,
        new THREE.MeshBasicMaterial({
          color: '#0000FF'
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

var light = new THREE.SpotLight();
light.position.set( 170, 330, -160 );
scene.add(light);

var sequence;

////////////////////////////////

var terrain, agents = [];
var recast = require('../lib/recast');

recast.setGLContext(renderer.context);

function render () {
    renderer.render(scene, camera);
}

function raycast (e, callback) {
    var x = e.pageX - $(e.currentTarget).offset().left;
    var y = e.pageY - $(e.currentTarget).offset().top;

    mouse.x = ( x / e.currentTarget.width ) * 2 - 1;
    mouse.y = - ( y / e.currentTarget.height ) * 2 + 1;

    // update the picking ray with the camera and mouse position
    raycaster.setFromCamera(mouse, camera);

    // calculate objects intersecting the picking ray
    var intersects = raycaster.intersectObjects(terrain.children, true);

    if (intersects.length) {
        var point = intersects[0].point;
        callback(point);
    }
}

function addObstacle (x, y, z) {
    var radius = 3;
    var obstacleMesh = new THREE.Mesh(
        new THREE.CylinderGeometry(radius, radius, 2),
        new THREE.MeshBasicMaterial({
            color: 0xff0000,
            shading: THREE.FlatShading,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.6,
            overdraw: true
        })
    );
    obstacleMesh.position.set(x, y, z);
    scene.add(obstacleMesh);
    recast.addTempObstacle(x, y, z, radius);
}

var omlpair = {};
function addOffMeshLink (x, y, z) {

    var point = { x:x, y:y, z:z };

    if (! omlpair.from) {
        omlpair.from = point;

    } else if (! omlpair.to) {
        omlpair.to = point;
        recast.addOffMeshConnection(
            omlpair.from.x, omlpair.from.y, omlpair.from.z,
            omlpair.to.x,   omlpair.to.y,   omlpair.to.z,
            3, 1);

        var start = new THREE.Vector3(omlpair.from.x, omlpair.from.y, omlpair.from.z);
        var end   = new THREE.Vector3(omlpair.to.x,   omlpair.to.y,   omlpair.to.z);

        scene.add(createSpline(start, end, 3));

        omlpair = {};
    }
}

var splineMaterial = new THREE.LineBasicMaterial({
  color: 0x00ff00,
  transparent: true,
  opacity: 0.6,
  linewidth: 5,
  vertexColors: THREE.VertexColors
});

function createSpline(start, end, elevation) {

    var geometry = new THREE.Geometry();
    geometry.vertices.push(start);
    geometry.vertices.push(new THREE.Vector3(
        (start.x + end.x)  / 2,
        ((start.y + end.y) / 2) + elevation,
        (start.z + end.z)  / 2
    ));
    geometry.vertices.push(end);

    return new THREE.Line(geometry, splineMaterial);
}



// Check our library is here
exports['recast is present'] = function(test) {
    test.ok(recast, 'recast should be an object');
    test.done();
};

// Check file loading
exports['handle an agent'] = function(test) {
    test.expect(11);

    recast.set_cellSize(0.3);
    recast.set_cellHeight(0.2);
    recast.set_agentHeight(0.8);
    recast.set_agentRadius(0.2);
    recast.set_agentMaxClimb(4.0);
    recast.set_agentMaxSlope(30.0);

    var stats = new Stats();
    stats.setMode(1); // 0: fps, 1: ms
    stats.domElement.style.position = 'absolute';
    stats.domElement.style.right = '0px';
    stats.domElement.style.bottom = '0px';
    document.body.appendChild( stats.domElement );
    stats.setMode(0);
    stats.begin();


    var loader = new THREE.OBJLoader();
    loader.load('nav_test.obj', function(object){
        terrain = object;
        object.traverse(function(child) {
            if (child instanceof THREE.Mesh) {
                child.material.side = THREE.DoubleSide;
                // child.material.shading = THREE.FlatShading;
            }
        } );
        scene.add(object);
    });


    /**
     * Load an .OBJ file
     */
    recast.OBJLoader('nav_test.obj', function(){

        // recast.buildTiled();
        // recast.loadTileMesh('./navmesh.dist.bin', recast.cb(function(){
        recast.loadTileCache('./tilecache.dist.bin', recast.cb(function(){

        recast.initCrowd(1000, 1.0);

        recast.vent.on('update', function (agents) {
            for (var i = 0; i < agents.length; i++) {
                var agent = agents[i];

                var angle = Math.atan2(- agent.velocity.z, agent.velocity.x);
                if (Math.abs(agentsObjects[agent.idx].rotation.y - angle) > 0) {
                    agentsObjects[agent.idx].rotation.y = angle;
                }

                agentsObjects[agent.idx].position.set(
                    agent.position.x,
                    agent.position.y,
                    agent.position.z
                );
            }
        });

        /**
         * Add some agents
         */
        for (var i = 0; i < agentsObjects.length; i++) {
            agents.push(recast.addAgent({
                position: {
                    x: -25.8850,
                    y: -1.64166,
                    z: -5.41350
                },
                radius: 0.8,
                height: 0.5,
                maxAcceleration: 1.0,
                maxSpeed: 2.0,
                updateFlags: 0, // && recast.CROWD_OBSTACLE_AVOIDANCE, // & recast.CROWD_ANTICIPATE_TURNS & recast.CROWD_OPTIMIZE_TOPO & recast.CROWD_SEPARATION,
                separationWeight: 20.0
            }));
        }

        var routes;

        var last = new Date().getTime();
        var animate = function animate (time) {

            setTimeout(function () {
                recast.crowdUpdate(0.1);
                recast.crowdGetActiveAgents();
            }, 0);

            window.requestAnimationFrame(animate);

            last = time;
            render();

            if (stats) stats.update();
        };

        animate(new Date().getTime());

        sequence = function() {
            document.getElementById('sequence').style.display = 'none';
            routes = 0;
            goAway();
        };

        var goAway = function(){
            for (var i = 0; i < agentsObjects.length; i++) {
                (function (i) {
                    recast.getRandomPoint(recast.cb(function(pt2x, pt2y, pt2z){
                        recast.crowdRequestMoveTarget(i, pt2x, pt2y, pt2z);
                        if (++routes < MAX_HOPS) {
                            test.ok(true, 'route ' + routes + ': to ' + Math.round(pt2x, 2) + ',' + Math.round(pt2y, 2)+ ',' + Math.round(pt2z, 2));
                            setTimeout(goAway, 8000 * Math.random());
                        } else {
                            document.getElementById('sequence').style.display = 'block';
                            // test.done();
                        }
                    }));
                })(i);
            }
        };

        sequence();
      }));
    });
};

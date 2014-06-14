/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
/*jshint onevar: false, indent:4 */
/*global exports: true, require: true, THREE: true, Stats: true */
'use strict';

var renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setSize(document.body.clientWidth * 0.8, document.body.clientHeight * 0.8);
renderer.domElement.style.position = 'absolute';
renderer.domElement.style.bottom = 0;
renderer.domElement.style.right = 0;
document.body.appendChild(renderer.domElement);
renderer.setClearColorHex(0xFFFFFF, 1.0);
renderer.clear();

var width = renderer.domElement.width;
var height = renderer.domElement.height;
var camera = new THREE.PerspectiveCamera( 45, width / height, 1, 10000);
camera.position.y = 100;
camera.position.z = 100;

var controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.addEventListener('change', function(){
    renderer.render(scene, camera);
});

var agent = new THREE.Object3D();
var agentBody = new THREE.Mesh(
    new THREE.CylinderGeometry(1, 1, 2),
    new THREE.MeshBasicMaterial({
      color: '#FF0000'
    })
);
agentBody.position.y = 1;
agent.add(agentBody);

var scene = new THREE.Scene();

var navigationMesh;

var light = new THREE.SpotLight();
light.position.set( 170, 330, -160 );
scene.add(light);

renderer.render(scene, camera);

////////////////////////////////

var recast = require('../lib/recast');

// Check our library is here
exports['recast is present'] = function(test) {
    test.ok(recast, 'recast should be an object');
    test.done();
};

// Check our methods are here
exports['our methods are present'] = function(test) {
    test.ok(recast.set_cellSize, 'set_cellSize');
    test.ok(recast.set_cellHeight, 'set_cellHeight');
    test.ok(recast.set_agentHeight, 'set_agentHeight');
    test.ok(recast.set_agentRadius, 'set_agentRadius');
    test.ok(recast.set_agentMaxClimb, 'set_agentMaxClimb');
    test.ok(recast.set_agentMaxSlope, 'set_agentMaxSlope');

    test.ok(recast.build, 'build');
    test.ok(recast.initCrowd, 'initCrowd');
    test.ok(recast.initWithFileContent, 'initWithFileContent');
    test.ok(recast.findNearestPoint, 'findNearestPoint');
    test.ok(recast.findPath, 'findPath');
    test.ok(recast.getRandomPoint, 'getRandomPoint');

    test.ok(recast.addCrowdAgent, 'addCrowdAgent');
    test.ok(recast.updateCrowdAgentParameters, 'updateCrowdAgentParameters');
    test.ok(recast.requestMoveVelocity, 'requestMoveVelocity');
    test.ok(recast.removeCrowdAgent, 'removeCrowdAgent');
    test.ok(recast.crowdRequestMoveTarget, 'crowdRequestMoveTarget');
    test.ok(recast.crowdUpdate, 'crowdUpdate');
    test.ok(recast.crowdGetActiveAgents, 'crowdGetActiveAgents');
    test.done();
};

// Check file loading
exports['handle an agent'] = function(test) {
    test.expect(10);

    recast.set_cellSize(0.5);
    recast.set_cellHeight(0.8);
    recast.set_agentHeight(1.0);
    recast.set_agentRadius(0.2);
    recast.set_agentMaxClimb(4.0);
    recast.set_agentMaxSlope(30.0);

    /*
    recast.settings({
        cellSize: 2.0,
        cellHeight: 1.5,
        agentHeight: 2.0,
        agentRadius: 0.2,
        agentMaxClimb: 4.0,
        agentMaxSlope: 30.0
    });
    */


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
        scene.add(object);
    });


    /**
     * Load an .OBJ file
     */
    recast.OBJLoader('nav_test.obj', function(){

        /**
         * Get navmesh geometry and draw it
         */
        recast.getNavMeshVertices(recast.cb(function (vertices) {

            navigationMesh = new THREE.Object3D();
            var materials = [ new THREE.MeshNormalMaterial() ];

            for (var i = 0; i < vertices.length; i++) {
                if (!vertices[i+2]) { break; }

                var geometry = new THREE.ConvexGeometry([
                    new THREE.Vector3(   vertices[i].x,   vertices[i].y,   vertices[i].z ), 
                    new THREE.Vector3( vertices[i+1].x, vertices[i+1].y, vertices[i+1].z ),
                    new THREE.Vector3( vertices[i+2].x, vertices[i+2].y, vertices[i+2].z )
                ]);

                var child = THREE.SceneUtils.createMultiMaterialObject(geometry, materials);
                navigationMesh.add(child);

                i += 2;
            }

            scene.add(navigationMesh);

            renderer.render(scene, camera);
        }));

        recast.vent.on('update', function (agents) {
            var pos = agents[0].position;
            agent.position.set(pos.x, pos.y, pos.z);
        });

        scene.add(agent);

        /**
         * Add an agent, retain its ID
         */
        var id = recast.addAgent({
            position: {
                x: -25.8850,
                y: -1.64166,
                z: -5.41350
            },
            radius: 0.3,
            height: 0.5,
            maxAcceleration: 1.0,
            maxSpeed: 2.0,
            updateFlags: 0,
            separationWeight: 10.0
        });

        var last = new Date().getTime();
        var animate = function animate (time) {
            recast.crowdUpdate(0.1);
            recast.crowdGetActiveAgents();

            window.requestAnimationFrame(animate);

            last = time;
            renderer.render(scene, camera);
            if (stats) stats.update();
        };

        animate(new Date().getTime());

        var routes = 0;
        var goAway = function(){
            recast.getRandomPoint(recast.cb(function(pt2x, pt2y, pt2z){
                recast.crowdRequestMoveTarget(id, pt2x, pt2y, pt2z);
                if (++routes < 10) setTimeout(goAway, 2000);
                test.ok(true, 'route ' + routes + ': to ' + pt2x + ',' + pt2y + ',' + pt2z);
            }));
        };

        goAway();
    });
};

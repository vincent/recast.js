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
renderer.setSize(document.body.clientWidth * 0.6, document.body.clientHeight * 0.8);
renderer.domElement.style.position = 'absolute';
renderer.domElement.style.bottom = 0;
renderer.domElement.style.right = 0;
document.body.appendChild(renderer.domElement);
renderer.setClearColorHex(0xFFFFFF, 1.0);
renderer.clear();

var width = renderer.domElement.width;
var height = renderer.domElement.height;
var camera = new THREE.PerspectiveCamera( 45, width / height, 1, 10000);
camera.position.y = 50;
camera.position.z = 50;

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

var light = new THREE.SpotLight();
light.position.set( 170, 330, -160 );
scene.add(light);

var navigationMesh, sequence;

var createSpline = function(start, end, elevation) {

    var geometry = new THREE.Geometry();
    geometry.vertices.push(start);
    geometry.vertices.push(new THREE.Vector3(
        (start.x + end.x)  / 2,
        ((start.y + end.y) / 2) + elevation,
        (start.z + end.z)  / 2
    ));
    geometry.vertices.push(end);

    var material = new THREE.LineBasicMaterial({
      color: 0x00ff00,
      transparent: true,
      opacity: 0.6,
      linewidth: 5,
      vertexColors: THREE.VertexColors
    });

    return new THREE.Line(geometry, material);
};

////////////////////////////////

var terrain, agents = [];
var debugDraw = {};
var recast = require('../lib/recast');

recast.setGLContext(renderer.context);

function render () {
    renderer.render(scene, camera);

    if (debugDraw.NavMesh)              { recast.drawObject('NavMesh');             }                        
    if (debugDraw.NavMeshPortals)       { recast.drawObject('NavMeshPortals');      }          
    if (debugDraw.RegionConnections)    { recast.drawObject('RegionConnections');   }    
    if (debugDraw.RawContours)          { recast.drawObject('RawContours');         }                
    if (debugDraw.Contours)             { recast.drawObject('Contours');            }                      
    if (debugDraw.HeightfieldSolid)     { recast.drawObject('HeightfieldSolid');    }      
    if (debugDraw.HeightfieldWalkable)  { recast.drawObject('HeightfieldWalkable'); }
}

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
    test.expect(11);

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
            }
        } );
        scene.add(object);
    });


    /**
     * Load an .OBJ file
     */
    recast.OBJLoader('nav_test.obj', function(){

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

                var color = agent.neighbors * 128;
                agentsObjects[agent.idx].children[0].material.color.set(color, color, color);
            }
        });

        /**
         * Add some offmesh connections
         */
        var offMeshConnections = [
            {from:{ x:-6.741, y:-2.26, z: 27.167}, to:{ x:41.671, y:7.930, z: 19.195}},
            {from:{ x:47.776, y:-1.51, z:  -5.48}, to:{ x:-15.44, y:-2.26, z: 29.099}},
            {from:{ x:-22.21, y:-2.26, z: 27.166}, to:{ x:-21.11, y:-2.26, z: 26.584}},
            {from:{ x:-21.18, y:-2.26, z:29.6698}, to:{ x:21.452, y:-2.26, z: -14.40}},
            {from:{ x:-24.66, y:-2.26, z: -17.62}, to:{ x:14.015, y:9.311, z: 24.460}}
        ];

        for (var o = 0; o < offMeshConnections.length; o++) {
            var pair = offMeshConnections[o];

            recast.addOffMeshConnection(pair.from.x, pair.from.y, pair.from.z,
                                        pair.to.x,   pair.to.y,   pair.to.z,
                                        3, 1);

            var start = new THREE.Vector3(pair.from.x, pair.from.y, pair.from.z);
            var end   = new THREE.Vector3(pair.to.x,   pair.to.y,   pair.to.z);

            scene.add(createSpline(start, end, 3));
        }

        recast.buildSolo();
        recast.initCrowd(1000, 1.0);    

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
            window.requestAnimationFrame(animate);

            recast.crowdUpdate(0.1);
            recast.crowdGetActiveAgents();

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
                (function (agent) {
                    recast.getRandomPoint(recast.cb(function(pt2x, pt2y, pt2z){
                        recast.crowdRequestMoveTarget(agent, pt2x, pt2y, pt2z);
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
    });
};

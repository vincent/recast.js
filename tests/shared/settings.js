export default function settings(recast) {
  recast.set_cellSize(0.3);
  recast.set_cellHeight(0.2);
  recast.set_agentHeight(0.8);
  recast.set_agentRadius(0.2);
  recast.set_agentMaxClimb(4.0);
  recast.set_agentMaxSlope(30.0);
}

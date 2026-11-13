const fs = require('fs');
try {
 const input=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
 const nodes=input.nodes, ids=new Set(nodes.map(n=>n.id));
 const edges=input.edges.filter(e=>ids.has(e.source)&&ids.has(e.target));
 const ins=new Map(nodes.map(n=>[n.id,new Set()])),outs=new Map(nodes.map(n=>[n.id,new Set()]));
 for(const e of edges){ins.get(e.target).add(e.source);outs.get(e.source).add(e.target);}
 const fanInRanking=nodes.map(n=>({id:n.id,name:n.name,fanIn:ins.get(n.id).size})).sort((a,b)=>b.fanIn-a.fanIn||a.id.localeCompare(b.id));
 const fanOutRanking=nodes.map(n=>({id:n.id,name:n.name,fanOut:outs.get(n.id).size})).sort((a,b)=>b.fanOut-a.fanOut||a.id.localeCompare(b.id));
 const highOut=new Set(fanOutRanking.slice(0,Math.ceil(nodes.length*.1)).map(n=>n.id));
 const lowIn=new Set([...fanInRanking].sort((a,b)=>a.fanIn-b.fanIn||a.id.localeCompare(b.id)).slice(0,Math.ceil(nodes.length*.25)).map(n=>n.id));
 const entryNames=new Set(['index.ts','index.js','main.ts','main.js','app.ts','app.js','server.ts','server.js','mod.rs','main.go','main.py','main.rs','manage.py','app.py','wsgi.py','asgi.py','run.py','__main__.py','Application.java','Main.java','Program.cs','config.ru','index.php','App.swift','Application.kt','main.cpp','main.c']);
 const candidates=nodes.map(n=>{
  let score=0;
  if(n.type==='document'){if(n.filePath==='README.md')score+=5;else if(!n.filePath.includes('/')&&n.filePath.endsWith('.md'))score+=2;}
  if(n.type==='file'){if(entryNames.has(n.name))score+=3;if(n.filePath.split('/').length<=2)score++;if(highOut.has(n.id))score++;if(lowIn.has(n.id))score++;}
  return {id:n.id,score,name:n.name,summary:n.summary,type:n.type};
 }).sort((a,b)=>b.score-a.score||a.id.localeCompare(b.id));
 const start=candidates.find(n=>n.type==='file');
 const order=start?[start.id]:[],depthMap=start?{[start.id]:0}:{},byDepth={};
 const structural=edges.filter(e=>['imports','calls'].includes(e.type));
 for(let cursor=0;cursor<order.length;cursor++)for(const e of structural.filter(e=>e.source===order[cursor]))if(!(e.target in depthMap)){depthMap[e.target]=depthMap[e.source]+1;order.push(e.target);}
 for(const id of order)(byDepth[depthMap[id]]??=[]).push(id);
 const select=types=>nodes.filter(n=>types.includes(n.type)).map(({id,name,type,summary})=>({id,name,type,summary}));
 const structuralAdj=new Map(nodes.map(n=>[n.id,new Set()]));for(const e of structural)structuralAdj.get(e.source).add(e.target);
 const clusterMap=new Map();
 for(const e of structural){if(!structuralAdj.get(e.target).has(e.source))continue;const set=new Set([e.source,e.target]);
  for(const n of nodes){if(set.size>=5)break;if(set.has(n.id))continue;let linked=0;for(const member of set)if(structuralAdj.get(member).has(n.id)||structuralAdj.get(n.id).has(member))linked++;if(linked>=2)set.add(n.id);}
  const members=[...set].sort();const key=members.join('|');clusterMap.set(key,{nodes:members,edgeCount:edges.filter(e=>set.has(e.source)&&set.has(e.target)).length});
 }
 const output={scriptCompleted:true,entryPointCandidates:candidates.slice(0,5),fanInRanking:fanInRanking.slice(0,20),fanOutRanking:fanOutRanking.slice(0,20),bfsTraversal:{startNode:start?.id,order,depthMap,byDepth},nonCodeFiles:{documentation:select(['document']),infrastructure:select(['service','pipeline','resource']),data:select(['table','schema','endpoint']),config:select(['config'])},clusters:[...clusterMap.values()].sort((a,b)=>b.edgeCount-a.edgeCount).slice(0,10),layers:{count:input.layers.length,list:input.layers},nodeSummaryIndex:Object.fromEntries(nodes.map(n=>[n.id,{name:n.name,type:n.type,summary:n.summary}])),totalNodes:nodes.length,totalEdges:edges.length,excludedSubFileEdges:input.edges.length-edges.length};
 fs.writeFileSync(process.argv[3],JSON.stringify(output,null,2)+'\n');
 console.log(`Topology: ${nodes.length} file-level nodes, ${edges.length} file-level edges; entry ${start?.id}; ${output.clusters.length} mutual clusters.`);
}catch(e){console.error(e);process.exit(1);}

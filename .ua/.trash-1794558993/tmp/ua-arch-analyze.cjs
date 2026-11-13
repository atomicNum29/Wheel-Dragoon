const fs=require('fs');
try {
 const {fileNodes,importEdges,allEdges}=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
 const byId=new Map(fileNodes.map(n=>[n.id,n]));
 const segments=fileNodes.map(n=>n.filePath.split('/').slice(0,-1));
 let common=segments[0]||[];
 for(const s of segments){let i=0;while(i<common.length&&i<s.length&&common[i]===s[i])i++;common=common.slice(0,i);}
 const flat=segments.every(s=>s.length===common.length);
 const groupOf=n=>{const tail=n.filePath.split('/').slice(common.length);return tail.length>1?tail[0]:(flat?(n.filePath.match(/\.[^.\/]+$/)?.[0]||'root'):'root');};
 const directoryGroups={},nodeTypeGroups={},fileFanIn={},fileFanOut={},adjacency={},groupImports={},groupImportedBy={},groupById={};
 for(const n of fileNodes){const g=groupOf(n);groupById[n.id]=g;(directoryGroups[g]??=[]).push(n.id);(nodeTypeGroups[n.type]??=[]).push(n.id);fileFanIn[n.id]=0;fileFanOut[n.id]=0;adjacency[n.id]=[];groupImports[g]??=new Set();groupImportedBy[g]??=new Set();}
 const pairs=new Map();
 for(const e of importEdges){if(!byId.has(e.source)||!byId.has(e.target))continue;fileFanOut[e.source]++;fileFanIn[e.target]++;adjacency[e.source].push(e.target);const a=groupById[e.source],b=groupById[e.target];pairs.set(a+'|'+b,(pairs.get(a+'|'+b)||0)+1);if(a!==b){groupImports[a].add(b);groupImportedBy[b].add(a);}}
 const cross=new Map(),nonCodeConnections=[];
 for(const e of allEdges){const a=byId.get(e.source),b=byId.get(e.target);if(!a||!b)continue;const key=a.type+'|'+b.type+'|'+e.type;cross.set(key,(cross.get(key)||0)+1);if(a.type!=='file'&&b.type==='file')nonCodeConnections.push(e);}
 const intraGroupDensity={};
 for(const g of Object.keys(directoryGroups)){const internalEdges=importEdges.filter(e=>groupById[e.source]===g&&groupById[e.target]===g).length;const totalEdges=importEdges.filter(e=>groupById[e.source]===g||groupById[e.target]===g).length;intraGroupDensity[g]={internalEdges,totalEdges,density:totalEdges?internalEdges/totalEdges:0};}
 const patternMatches={};
 const patterns={api:['routes','api','controllers','endpoints','handlers','serializers','routers','blueprints'],service:['services','core','lib','domain','logic','signals','internal','composables','mailers','jobs','channels'],data:['models','db','data','persistence','repository','entities','migrations','sql','database','schema','entity'],ui:['components','views','pages','ui','layouts','screens'],middleware:['middleware','plugins','interceptors','guards'],utility:['utils','helpers','common','shared','tools','templatetags','pkg'],config:['config','constants','env','settings','management','commands'],test:['__tests__','test','tests','spec','specs'],types:['types','interfaces','schemas','contracts','dtos','dto','request','response'],hooks:['hooks'],state:['store','state','reducers','actions','slices'],assets:['assets','static','public'],entry:['cmd','bin'],documentation:['docs','documentation','wiki'],infrastructure:['deploy','deployment','infra','infrastructure','k8s','kubernetes','helm','charts','terraform','tf','docker'],'ci-cd':['.github','.gitlab','.circleci']};
 for(const g of Object.keys(directoryGroups))patternMatches[g]=Object.entries(patterns).find(([,v])=>v.includes(g))?.[0]||null;
 const files=fileNodes.map(n=>n.filePath);
 const filePatternMatches=Object.fromEntries(fileNodes.map(n=>[n.filePath,/\.(test|spec)\.|(^|\/)test_|_test\.|Test\.(java|php)|Tests\.cs/.test(n.filePath)?'test':/(^|\/)(README(?:\.|$)|[^/]+\.(md|rst)$)/i.test(n.filePath)?'documentation':n.type==='config'||/(^|\/)(platformio\.ini|pyproject\.toml|\.python-version)$/.test(n.filePath)?'config':/(^|\/)(main\.(cpp|go|rs)|index\.(ts|js)|__init__\.py|manage\.py|Application\.java|Program\.cs)$/.test(n.filePath)?'entry':n.filePath.endsWith('.hpp')?'types':null]));
 const deploymentTopology={hasDockerfile:files.some(p=>/(^|\/)Dockerfile/.test(p)),hasCompose:files.some(p=>/(docker-compose|compose)\.ya?ml$/.test(p)),hasK8s:files.some(p=>/(^|\/)(k8s|kubernetes)\//.test(p)),hasTerraform:files.some(p=>/\.tf$/.test(p)),hasCI:files.some(p=>/\.github\/workflows|\.gitlab-ci|Jenkinsfile|\.circleci/.test(p)),infraFiles:fileNodes.filter(n=>['service','pipeline','resource'].includes(n.type)).map(n=>n.filePath)};
 const dataPipeline={schemaFiles:fileNodes.filter(n=>n.type==='schema').map(n=>n.filePath),migrationFiles:files.filter(p=>/migrations?\//.test(p)),dataModelFiles:fileNodes.filter(n=>(n.tags||[]).some(t=>['data-model','데이터모델'].includes(t))).map(n=>n.filePath),apiHandlerFiles:fileNodes.filter(n=>(n.tags||[]).some(t=>['api-handler','endpoint'].includes(t))).map(n=>n.filePath)};
 const docs=new Set(fileNodes.filter(n=>n.type==='document'||/(^|\/)README(\.|$)/i.test(n.filePath)).map(n=>n.id));
 const documented=new Set([...docs].map(id=>groupById[id]));
 for(const e of allEdges)if(docs.has(e.source)&&e.type==='documents'&&groupById[e.target])documented.add(groupById[e.target]);
 const groups=Object.keys(directoryGroups);
 const dependencyDirection=[];
 for(let i=0;i<groups.length;i++)for(let j=i+1;j<groups.length;j++){const a=groups[i],b=groups[j],ab=pairs.get(a+'|'+b)||0,ba=pairs.get(b+'|'+a)||0;if(ab!==ba)dependencyDirection.push({dependent:ab>ba?a:b,dependsOn:ab>ba?b:a});}
 const output={scriptCompleted:true,commonPrefix:common.join('/'),directoryGroups,nodeTypeGroups,adjacency,groupImports:Object.fromEntries(Object.entries(groupImports).map(([k,v])=>[k,[...v]])),groupImportedBy:Object.fromEntries(Object.entries(groupImportedBy).map(([k,v])=>[k,[...v]])),crossCategoryEdges:[...cross].map(([k,count])=>{const [fromType,toType,edgeType]=k.split('|');return{fromType,toType,edgeType,count};}),nonCodeConnections,interGroupImports:[...pairs].filter(([k])=>k.split('|')[0]!==k.split('|')[1]).map(([k,count])=>{const[from,to]=k.split('|');return{from,to,count};}),intraGroupDensity,patternMatches,deploymentTopology,dataPipeline,docCoverage:{groupsWithDocs:documented.size,totalGroups:groups.length,coverageRatio:documented.size/groups.length,undocumentedGroups:groups.filter(g=>!documented.has(g))},dependencyDirection,fileStats:{totalFileNodes:fileNodes.length,filesPerGroup:Object.fromEntries(Object.entries(directoryGroups).map(([k,v])=>[k,v.length])),nodeTypeCounts:Object.fromEntries(Object.entries(nodeTypeGroups).map(([k,v])=>[k,v.length]))},fileFanIn,fileFanOut};
 output.filePatternMatches=filePatternMatches;
 fs.writeFileSync(process.argv[3],JSON.stringify(output,null,2));
}catch(e){console.error(e);process.exit(1);}

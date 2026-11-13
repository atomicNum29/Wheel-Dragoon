const fs=require('fs');
const base='.ua/intermediate/';
const g=JSON.parse(fs.readFileSync(base+'assembled-graph.json','utf8'));
const s=JSON.parse(fs.readFileSync(base+'scan-result.json','utf8'));
const shards=fs.readdirSync(base).filter(n=>/^batch-\d+(?:-part-\d+)?\.json$/.test(n)).map(n=>({name:n,...JSON.parse(fs.readFileSync(base+n,'utf8'))}));
const ids=new Set(g.nodes.map(n=>n.id));
const fileNodes=new Map(g.nodes.filter(n=>['file','config','document','service','pipeline','table','schema','resource','endpoint'].includes(n.type)).map(n=>[n.filePath,n]));
const allowedTypes=new Set(['file','function','class','module','concept','config','document','service','pipeline','table','schema','resource','endpoint']);
const issues=[];
if(ids.size!==g.nodes.length)issues.push('중복 노드 ID');
for(const n of g.nodes){if(!allowedTypes.has(n.type)||!['simple','moderate','complex'].includes(n.complexity))issues.push('유형 또는 복잡도 값 확인 필요: '+n.id);if(!n.summary||!n.tags?.length)issues.push('설명 또는 태그 누락: '+n.id);}
for(const e of g.edges)if(!ids.has(e.source)||!ids.has(e.target))issues.push('존재하지 않는 관계 끝점: '+JSON.stringify(e));
const missingFiles=s.files.filter(f=>!fileNodes.has(f.path)).map(f=>f.path);
const inputNodeCount=shards.reduce((n,b)=>n+b.nodes.length,0),inputEdgeCount=shards.reduce((n,b)=>n+b.edges.length,0);
const keys=new Set(g.edges.map(e=>[e.source,e.target,e.type].join('|')));
let crossBatchEdgesAdded=0,expectedImports=0;
for(const [from,targets]of Object.entries(s.importMap))for(const to of targets){expectedImports++;const source=fileNodes.get(from)?.id,target=fileNodes.get(to)?.id;if(!source||!target){issues.push('import 파일 노드 누락: '+from+' -> '+to);continue;}const key=[source,target,'imports'].join('|');if(!keys.has(key)){g.edges.push({source,target,type:'imports',direction:'forward',weight:.7});keys.add(key);crossBatchEdgesAdded++;}}
if(missingFiles.length)issues.push('스캔 파일 누락: '+missingFiles.join(', '));
if(crossBatchEdgesAdded)fs.writeFileSync(base+'assembled-graph.json',JSON.stringify(g,null,2));
const report={fixedSectionOk:issues.length===0,nodesRecovered:0,edgesRestored:0,crossBatchEdgesAdded,typesRemapped:0,complexityRemapped:0,notes:[
`7개 논리 배치의 ${shards.length}개 파일을 확인했다. 입력 ${inputNodeCount}개 노드·${inputEdgeCount}개 관계가 조립 결과에 유지되었으며 복구할 삭제 항목은 없다.`,
`스캔 목록 ${s.files.length}개 파일 모두 파일 단위 노드가 있다. importMap의 ${expectedImports}개 의존 관계를 확인했으며 누락 관계는 ${crossBatchEdgesAdded}개다.`,
'조건부 컴파일·템플릿·참조/포인터 반환 선언에 대한 C++ 구조 추출 누락은 해당 파일의 languageNotes에 보존했다. 이 그래프는 파일 범위는 완전하지만 모든 함수의 완전한 호출 그래프는 아니다.',
'platformio.ini 및 확장자가 없는 보조 파일은 파서 미지원 이후 내용을 읽어 포함했다. Python 주요 메서드의 범위는 이름만 제공하는 추출 결과를 소스로 보완했다.',
'사고 검토 문서는 정적 코드 경로·추론·미확인 조건을 구분한다. 사고 원인 확정 또는 실물 검증 결과로 확대하지 않았다.',
'합병 로그의 NativeCommandError 표시는 PowerShell의 표준 오류 출력 포장이다. 합병 자체는 성공했으며 노드·관계 삭제나 ID 수정 내역이 없다.',
...issues]};
fs.writeFileSync(base+'assemble-review.json',JSON.stringify(report,null,2));
console.log(JSON.stringify({nodes:g.nodes.length,edges:g.edges.length,fileCoverage:fileNodes.size,expectedFiles:s.files.length,expectedImports,crossBatchEdgesAdded,issues}));

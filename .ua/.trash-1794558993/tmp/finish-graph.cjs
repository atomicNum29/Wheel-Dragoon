const fs = require('fs');
const path = require('path');
const root = process.cwd();
const dir = path.join(root, '.ua');
const inter = path.join(dir, 'intermediate');
const read = name => JSON.parse(fs.readFileSync(path.join(inter, name), 'utf8').replace(/^\uFEFF/, ''));
const write = (name, value) => fs.writeFileSync(path.join(inter, name), JSON.stringify(value, null, 2) + '\n');
const scan = read('scan-result.json');
const graph = read('assembled-graph.json');
const fileTypes = new Set(['file','config','document','service','pipeline','table','schema','resource','endpoint']);
const files = graph.nodes.filter(n => fileTypes.has(n.type));
const nodeIds = new Set(graph.nodes.map(n => n.id));
const pathIndex = new Map(files.map(n => [n.filePath, n]));
const mode = process.argv[2];
if (mode === 'assemble') {
  let layers = read('layers.json');
  layers = Array.isArray(layers) ? layers : layers.layers;
  for (const [i, layer] of layers.entries()) {
    layer.id ||= `layer:group-${i + 1}`;
    layer.nodeIds = (layer.nodeIds || layer.nodes || []).map(x => typeof x === 'string' ? x : x.id)
      .map(x => nodeIds.has(x) ? x : pathIndex.get(x)?.id).filter(x => nodeIds.has(x));
    delete layer.nodes;
    if (!layer.name || !layer.description || !layer.nodeIds.length) throw new Error('Invalid layer ' + layer.id);
  }
  graph.version = '1.0.0';
  graph.project = {name:scan.name, languages:scan.languages, frameworks:scan.frameworks, description:scan.description,
    analyzedAt:new Date().toISOString(), gitCommitHash:process.argv[3]};
  graph.layers = layers;
  graph.tour = read('tour.normalized.json');
  for (const step of graph.tour) {
    if (!step.title || !step.description || !step.nodeIds?.length || !step.nodeIds.every(id => nodeIds.has(id))) throw new Error('Invalid tour');
  }
  const missing = scan.files.filter(f => !pathIndex.has(f.path));
  if (missing.length) throw new Error('Missing files: ' + missing.map(f => f.path).join(', '));
  write('assembled-graph.json', graph);
  write('fingerprint-input.json', {projectRoot:root, filePaths:scan.files.map(f => f.path), gitCommitHash:process.argv[3]});
  console.log(JSON.stringify({files:files.length,nodes:graph.nodes.length,edges:graph.edges.length,layers:layers.length,tour:graph.tour.length}));
} else if (mode === 'lesson-plan') {
  const tour = read('tour.normalized.json');
  const anchors = new Map();
  for (const step of tour) for (const id of step.nodeIds) {
    const node = files.find(n => n.id === id);
    if (node && !anchors.has(node.filePath)) anchors.set(node.filePath, step.order);
  }
  const details = [...anchors].map(([filePath,tourOrder]) => ({filePath,tourOrder}));
  const batches = [];
  for (let i = 0; i < details.length; i += 12) batches.push({batchIndex:batches.length+1,files:details.slice(i,i+12)});
  write('lesson-plan.json', {detailedFiles:details,batches});
  console.log(JSON.stringify(batches));
} else if (mode === 'lessons') {
  const plan = read('lesson-plan.json');
  const tour = read('tour.normalized.json');
  const orders = new Set(tour.map(t => t.order));
  const merged = {};
  const warnings = [];
  for (const batch of plan.batches) {
    const name = plan.batches.length === 1 ? 'code-lessons.json' : `code-lessons.detailed.${batch.batchIndex}.json`;
    const data = read(name);
    for (const [key,val] of Object.entries(data.files || data)) {
      if (!pathIndex.has(key)) { warnings.push('Unknown lesson file: '+key); continue; }
      if (merged[key]) { warnings.push('Duplicate lesson file: '+key); continue; }
      merged[key] = val;
    }
  }
  for (const f of plan.detailedFiles) if (!merged[f.filePath]) throw new Error('Missing detailed lesson: '+f.filePath);
  for (const node of files) {
    const entry = merged[node.filePath] ||= {overview:node.summary,sections:[]};
    entry.overview ||= node.summary;
    const total = fs.readFileSync(path.join(root,node.filePath),'utf8').split(/\r?\n/).length;
    let last = 0;
    entry.sections = (entry.sections || []).filter(s => {
      const r = s.lineRange;
      const valid = Array.isArray(r) && r.length===2 && r.every(Number.isInteger) && r[0]>=1 && r[0]<=r[1] && r[1]<=total && r[0]>last;
      if (valid) last=r[1]; else warnings.push('Invalid section: '+node.filePath);
      return valid;
    }).slice(0,10);
    for (const section of entry.sections) if (!orders.has(section.tourOrder)) delete section.tourOrder;
  }
  write('code-lessons.normalized.json',{version:1,generatedAt:new Date().toISOString(),files:merged});
  write('lesson-normalization.json',{detailed:Object.values(merged).filter(x=>x.sections.length).length,overviewOnly:Object.values(merged).filter(x=>!x.sections.length).length,warnings});
  console.log(JSON.stringify(read('lesson-normalization.json')));
} else if (mode === 'report') {
  const review = read('review.json');
  const assembly = read('assemble-review.json');
  const lessons = read('lesson-normalization.json');
  const categories = scan.files.reduce((a,f) => (a[f.fileCategory]=(a[f.fileCategory]||0)+1,a),{});
  const pairs = obj => Object.entries(obj).map(([k,v]) => `${k} ${v}`).join(', ');
  const lines = [
    '# Wheel-Dragoon 한국어 코드베이스 분석', '',
    '기준 커밋: `' + graph.project.gitCommitHash + '`',
    '생성 시각(UTC): ' + graph.project.analyzedAt, '',
    graph.project.description, '',
    'Python 프로젝트 메타데이터 이름은 `car-demo`이며 저장소 이름은 Wheel-Dragoon이다.', '',
    '## 범위', '',
    '- 분석: ' + files.length + '/' + scan.totalFiles + '개 파일 (' + pairs(categories) + ')',
    '- 사용자 승인 제외: `.pio/`, `.venv/`, `.vscode/`, `backups/`',
    '- 추가 제외 규칙 적용: ' + scan.filteredByIgnore + '개 파일. 기본 제외 파일은 이 숫자에 포함하지 않는다.',
    '- 스캐너가 포함한 `.ua/config.json`도 설정 노드로 유지했다.', '',
    '## 그래프', '',
    '- 노드 ' + review.stats.totalNodes + '개: ' + pairs(review.stats.nodeTypes),
    '- 관계 ' + review.stats.totalEdges + '개: ' + pairs(review.stats.edgeTypes),
    '- 학습 순서 ' + graph.tour.length + '단계',
    '- 파일 학습 설명: 상세 ' + lessons.detailed + '개, 개요 ' + lessons.overviewOnly + '개', '',
    '| 계층 | 파일 수 | 역할 |','| --- | ---: | --- |',
    ...graph.layers.map(l => `| ${l.name} | ${l.nodeIds.length} | ${l.description} |`), '',
    '## 검증과 한계', '',
    '- 생성 그래프의 필수 필드, 중복 ID, 관계 대상, 계층 배정, 학습 순서 참조를 확인했다. 오류 ' + review.issues.length + '건.',
    '- 프로젝트 테스트, 펌웨어 빌드, 업로드, 하드웨어 통신은 수행하지 않았다.',
    ...assembly.notes.filter(x => !x.includes('PowerShell')).map(x => '- ' + x),
    ...review.warnings.map(x => '- 그래프 경고: ' + x),
    ...lessons.warnings.map(x => '- 학습 설명 경고: ' + x), '',
    '## 학습 순서', '',
    ...graph.tour.map(s => `${s.order}. **${s.title}** — ${s.description}`), '',
    '산출물: `knowledge-graph.json`, `code-lessons.json`, `fingerprints.json`, `meta.json`.', ''
  ];
  fs.writeFileSync(path.join(dir,'analysis-report.ko.md'),lines.join('\n'));
  console.log(JSON.stringify({categories,...review.stats,lessons,layers:graph.layers.map(l=>l.name)}));
} else throw new Error('Unknown mode');

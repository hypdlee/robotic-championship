python3 - <<'PY'
import re
out=open('/home/claude/out/lab.html').read();i=[x.start() for x in re.finditer('<script>',out)][0];j=out.find('</script>',i)+9
t=open('template.html').read()
open('deployment-cases.html','w').write(t.replace('/*THREE*/',out[i:j]).replace('/*CAD*/',open('/home/claude/lab/genesis_lab.json').read()).replace('/*CORE*/',open('core.js').read()).replace('/*SIM*/',open('cases_core.js').read()))
PY

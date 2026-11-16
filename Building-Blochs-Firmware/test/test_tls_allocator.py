"""Host test for the production TLS allocator with simulated ESP heaps."""
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class TLSAllocatorTests(unittest.TestCase):
    def test_external_first_fallback_exhaustion_and_overflow(self):
        compiler = shutil.which('g++')
        if not compiler:
            self.skipTest('g++ is needed for the allocator host test')
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder/'esp_heap_caps.h').write_text('''#include <cstdlib>
#include <cstdint>
constexpr unsigned MALLOC_CAP_SPIRAM=1, MALLOC_CAP_8BIT=2, MALLOC_CAP_INTERNAL=4;
static unsigned calls=0, flags[2];
static bool external=true, internal=true;
static void *heap_caps_calloc(size_t n,size_t s,unsigned caps) {
    flags[calls++]=caps;
    return ((caps&MALLOC_CAP_SPIRAM)?external:internal) ? calloc(n,s) : nullptr;
}
''')
            (folder/'main.cpp').write_text('''#include <cassert>
#include "tls_allocator.h"
int main() {
    auto *p=static_cast<unsigned char *>(tlsCalloc(2,16384));
    assert(p && calls==1 && flags[0]==(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    for(unsigned i=0;i<32768;++i) assert(p[i]==0);
    free(p);
    calls=0;external=false;
    p=static_cast<unsigned char *>(tlsCalloc(1,16384));
    assert(p && calls==2 && flags[1]==(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
    free(p);
    calls=0;internal=false;
    assert(tlsCalloc(1,16384)==nullptr && calls==2);
    calls=0;
    assert(tlsCalloc(SIZE_MAX,2)==nullptr && calls==0);
}
''')
            executable = folder/'allocator-test.exe'
            env = dict(os.environ)
            env['PATH'] = str(Path(compiler).parent)+os.pathsep+env.get('PATH','')
            subprocess.run([compiler,'-std=c++14','-I',str(folder),'-I',str(ROOT/'src'),str(folder/'main.cpp'),'-o',str(executable)],check=True,env=env)
            subprocess.run([str(executable)],check=True,env=env)

if __name__ == '__main__':
    unittest.main()

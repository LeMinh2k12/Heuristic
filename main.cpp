#pragma GCC optimize("O3,unroll-loops,fast-math,inline")
#pragma GCC optimize("Ofast")
#pragma GCC optimize("no-stack-protector")
#pragma GCC optimize("omit-frame-pointer")
#pragma GCC target("avx2,bmi,bmi2,popcnt,lzcnt,abm")

#include <bits/stdc++.h>
using namespace std;

namespace FastIO {
constexpr int SZ = 1 << 16;
char in[SZ], out[SZ]; int ip = 0, il = 0, op = 0;
inline int gc() {
    if (ip == il) { il = (int)fread(in, 1, SZ, stdin); ip = 0; if (!il) return EOF; }
    return (unsigned char)in[ip++];
}
inline bool readInt(int &v) {
    int c; do { c = gc(); if (c == EOF) return false; } while (c <= ' ');
    int sign = 1; if (c == '-') { sign = -1; c = gc(); }
    v = 0; while (c >= '0' && c <= '9') { v = v * 10 + c - '0'; c = gc(); }
    v *= sign; return true;
}
inline void flush() { fwrite(out, 1, op, stdout); op = 0; }
inline void pc(char c) { if (op == SZ) flush(); out[op++] = c; }
inline void writeInt(int x) { if (x < 0) { pc('-'); x = -x; } char b[12]; int n = 0; do { b[n++] = char('0' + x % 10); x /= 10; } while (x); while (n) pc(b[--n]); }
}

// The given constraints are N=128, M=150, K=3.
constexpr int MAXM = 150, KMAX = 3, UNUSED = 16384;
constexpr double TIME_LIMIT = 1.80;
int N, M, K, T, gx[MAXM + 1], gy[MAXM + 1];
auto started = chrono::steady_clock::now();
inline double elapsed() { return chrono::duration<double>(chrono::steady_clock::now() - started).count(); }
inline int cell(int x, int y) { return x * N + y; }
inline int absi(int x) { return x < 0 ? -x : x; }
inline int md(int x, int y, int a, int b) { return absi(x-a) + absi(y-b); }
inline int clampi(int x, int lo, int hi) { return max(lo, min(hi, x)); }

// circles[] is sorted, with UNUSED marking a circle whose initial position
// has not yet been committed. The free circle really exists from time zero;
// its initial position will be supplied when we reconstruct the output.
struct Node {
    uint16_t circles[KMAX];
    uint16_t dep, center; // departure and used center; center=UNUSED for walk
    uint16_t dep2, center2; // optional second teleport on this leg
    uint16_t parent;
    uint8_t free_circle;
    int cost;
};
struct Candidate { Node node; int rank; };
vector<Node> layers[MAXM + 1];
vector<Node> best_path;
int best_cost;
bool direct_moves = false;
uint8_t coord_x[16384], coord_y[16384];
uint16_t axis_x[MAXM][128], axis_y[MAXM][128];
uint16_t two_x[MAXM][128*128], two_y[MAXM][128*128];
inline int axisTwo(int s,int g,int c1,int c2) {
    int l1=max(0,2*c1-(N-1)),h1=min(N-1,2*c1);
    int l2=max(0,2*c2-(N-1)),h2=min(N-1,2*c2);
    int a=2*c1-s, b=2*c2-g;
    int w=clampi(a,l1,h1), u=clampi(b,l2,h2);
    return absi(w-a)+absi(w-u)+absi(u-b);
}

inline int axisCost(int s, int g, int c) {
    int lo = max(0, 2*c - (N-1));
    int hi = min(N-1, 2*c);
    int t = 2*c - g;
    int a = min(s,t), b = max(s,t);
    return absi(s-t) + 2 * max(0, max(lo-b, a-hi));
}
inline int estimate(const Node &n, int depth) {
    if (depth >= M) return 0;
    static constexpr int weight[] = {100, 75, 55, 40};
    int xs[3], ys[3], px[6], py[6], cnt=0, pairs=0;
    while (cnt<K && n.circles[cnt]!=UNUSED) {
        xs[cnt]=coord_x[n.circles[cnt]];
        ys[cnt]=coord_y[n.circles[cnt]];
        ++cnt;
    }
    for (int a=0;a<cnt;++a) for (int b=0;b<cnt;++b) if (a!=b) {
        px[pairs]=xs[a]*128+xs[b];
        py[pairs]=ys[a]*128+ys[b];
        ++pairs;
    }
    int h = 0;
    for (int d = 0; d < 4 && depth+d < M; ++d) {
        int i = depth+d, sx = gx[i], sy = gy[i], tx = gx[i+1], ty = gy[i+1];
        int v = md(sx,sy,tx,ty);
        for (int j=0;j<cnt;++j)
            v=min(v,1+axis_x[i][xs[j]]+axis_y[i][ys[j]]);
        if (cnt<K) v=min(v,1+((sx^tx)&1)+((sy^ty)&1));
        if (d<2) for (int j=0;j<pairs;++j)
            v=min(v,2+two_x[i][px[j]]+two_y[i][py[j]]);
        h += weight[d] * v;
    }
    return h;
}
inline uint64_t key(const Node &n) {
    return uint64_t(n.circles[0]) | (uint64_t(n.circles[1]) << 15) | (uint64_t(n.circles[2]) << 30);
}
inline uint64_t hash64(uint64_t x) {
    x ^= x >> 30; x *= 0xbf58476d1ce4e5b9ULL;
    x ^= x >> 27; x *= 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}
inline int choices(int s, int g, int c, int next, int *a) {
    int lo = max(0, 2*c-(N-1)), hi = min(N-1,2*c), t = 2*c-g;
    int val[] = {s, t, next};
    int n = 0;
    for (int x : val) {
        x = clampi(x,lo,hi);
        bool seen = false; for (int i=0;i<n;++i) seen |= a[i] == x;
        if (!seen) a[n++] = x;
    }
    return n;
}
struct Pair { int p, q; };
inline int freeChoices(int s, int g, Pair *a) {
    int n=0;
    auto add = [&](int p,int q) {
        if (p<0 || p>=N || q<0 || q>=N || ((p+q)&1)) return;
        for (int i=0;i<n;++i) if (a[i].p==p && a[i].q==q) return;
        a[n++]={p,q};
    };
    if (((s^g)&1)==0) { add(s,g); add(s-2,g); add(s+2,g); }
    else { add(s-1,g); add(s+1,g); add(s,g-1); add(s,g+1); }
    return n;
}

bool runBeam(int width) {
    for (int i=0;i<=M;++i) layers[i].clear();
    Node root{}; for (int j=0;j<KMAX;++j) root.circles[j]=UNUSED;
    layers[0].push_back(root);
    vector<Candidate> cand; cand.reserve(width * 55);
    static uint64_t table[1 << 16];
    static uint32_t stamp[1 << 16];
    static uint64_t all_keys[1 << 20];
    static uint32_t all_stamp[1 << 20], all_index[1 << 20];
    memset(stamp, 0, sizeof(stamp));
    memset(all_stamp, 0, sizeof(all_stamp));
    uint32_t generation=0;
    for (int d=0; d<M; ++d) {
        if (elapsed() >= TIME_LIMIT) return false;
        cand.clear();
        const int sx=gx[d], sy=gy[d], tx=gx[d+1], ty=gy[d+1];
        const int walk=md(sx,sy,tx,ty);
        int nx = gx[min(M,d+2)], ny = gy[min(M,d+2)];
        auto push = [&](const Node &base, int parent, int idx, int px, int py,
                        int cx, int cy, int qx, int qy, bool freeCircle) {
            int add = md(sx,sy,px,py) + 1 + md(qx,qy,tx,ty);
            if (add > walk+8) return;
            Node z=base; z.cost+=add; z.parent=(uint16_t)parent;
            z.dep=(uint16_t)cell(px,py); z.center=(uint16_t)cell(cx,cy);
            z.free_circle=(uint8_t)freeCircle; z.center2=UNUSED;
            z.circles[idx]=(uint16_t)cell(px,py);
            sort(z.circles,z.circles+K);
            cand.push_back({z, 0});
        };
        for (int bi=0;bi<(int)layers[d].size();++bi) {
            const Node &base=layers[d][bi];
            Node z=base; z.cost+=walk; z.parent=(uint16_t)bi;
            z.dep=(uint16_t)cell(sx,sy); z.center=UNUSED; z.center2=UNUSED; z.free_circle=0;
            cand.push_back({z,0});
            for (int j=0;j<K;++j) {
                int c=base.circles[j];
                if (j && c==base.circles[j-1]) continue;
                if (c==UNUSED) {
                    Pair xs[4], ys[4]; int xn=freeChoices(sx,tx,xs), yn=freeChoices(sy,ty,ys);
                    for (int a=0;a<xn;++a) for (int b=0;b<yn;++b) {
                        int px=xs[a].p, py=ys[b].p, qx=xs[a].q, qy=ys[b].q;
                        push(base,bi,j,px,py,(px+qx)/2,(py+qy)/2,qx,qy,true);
                    }
                } else {
                    int cx=coord_x[c], cy=coord_y[c], xs[4], ys[4];
                    int xn=choices(sx,tx,cx,nx,xs), yn=choices(sy,ty,cy,ny,ys);
                    for (int a=0;a<xn;++a) for (int b=0;b<yn;++b) {
                        int px=xs[a],py=ys[b];
                        push(base,bi,j,px,py,cx,cy,2*cx-px,2*cy-py,false);
                    }
                }
            }
            // Two different established circles: first teleport from the
            // nearest feasible departure, then optimize the second departure.
            for (int j=0;j<K;++j) {
                int c1=base.circles[j];
                if (c1==UNUSED || (j && c1==base.circles[j-1])) continue;
                int cx1=coord_x[c1], cy1=coord_y[c1];
                int px=clampi(sx,max(0,2*cx1-(N-1)),min(N-1,2*cx1));
                int py=clampi(sy,max(0,2*cy1-(N-1)),min(N-1,2*cy1));
                int qx=2*cx1-px, qy=2*cy1-py;
                int first=md(sx,sy,px,py)+2;
                for (int k=0;k<K;++k) {
                    int c2=base.circles[k];
                    if (k==j || c2==UNUSED || (k && k-1!=j && c2==base.circles[k-1])) continue;
                    int cx2=coord_x[c2], cy2=coord_y[c2];
                    if (first + axisCost(qx,tx,cx2) + axisCost(qy,ty,cy2) > walk+5) continue;
                    int xs[4],ys[4];
                    int xn=choices(qx,tx,cx2,nx,xs), yn=choices(qy,ty,cy2,ny,ys);
                    for (int a=0;a<xn;++a) for (int b=0;b<yn;++b) {
                        int ux=xs[a], uy=ys[b], vx=2*cx2-ux, vy=2*cy2-uy;
                        int add=first+md(qx,qy,ux,uy)+md(vx,vy,tx,ty);
                        if (add>walk+5) continue;
                        Node t=base; t.cost+=add; t.parent=(uint16_t)bi;
                        t.dep=cell(px,py); t.center=c1;
                        t.dep2=cell(ux,uy); t.center2=c2; t.free_circle=0;
                        t.circles[j]=cell(px,py); t.circles[k]=cell(ux,uy);
                        sort(t.circles,t.circles+K);
                        cand.push_back({t,0});
                    }
                }
            }
            // Two consecutive reflections compose into a translation.  Try
            // departures along the entire optimal interval, not just near s.
            if (direct_moves) for (int j=0;j<K;++j) {
                int c1=base.circles[j];
                if (c1==UNUSED || (j && c1==base.circles[j-1])) continue;
                int cx1=coord_x[c1], cy1=coord_y[c1];
                for (int k=0;k<K;++k) {
                    int c2=base.circles[k];
                    if (k==j || c2==UNUSED || (k && k-1!=j && c2==base.circles[k-1])) continue;
                    int cx2=coord_x[c2], cy2=coord_y[c2];
                    int dx=2*(cx2-cx1), dy=2*(cy2-cy1);
                    int lx=max({0,2*cx1-(N-1),-dx});
                    int hx=min({N-1,2*cx1,N-1-dx});
                    int ly=max({0,2*cy1-(N-1),-dy});
                    int hy=min({N-1,2*cy1,N-1-dy});
                    if (lx>hx || ly>hy) continue;
                    int ax=clampi(sx,lx,hx), bx=clampi(tx-dx,lx,hx);
                    int ay=clampi(sy,ly,hy), by=clampi(ty-dy,ly,hy);
                    int xs[3]={ax,bx,(ax+bx)/2};
                    int ys[3]={ay,by,(ay+by)/2};
                    for (int a=0;a<3;++a) for (int b=0;b<3;++b) {
                        int px=xs[a], py=ys[b], qx=2*cx1-px, qy=2*cy1-py;
                        int add=md(sx,sy,px,py)+2+md(px+dx,py+dy,tx,ty);
                        if (add>walk+5) continue;
                        Node t=base; t.cost+=add; t.parent=(uint16_t)bi;
                        t.dep=cell(px,py); t.center=c1;
                        t.dep2=cell(qx,qy); t.center2=c2; t.free_circle=0;
                        t.circles[j]=cell(px,py); t.circles[k]=cell(qx,qy);
                        sort(t.circles,t.circles+K);
                        cand.push_back({t,0});
                    }
                }
            }
        }
        // Merge identical states before sorting: their future estimate is
        // identical, so only the cheapest parent can be useful.
        ++generation;
        bool deduplicated = cand.size() < (1U<<19);
        if (deduplicated) {
            size_t unique=0;
            for (size_t i=0;i<cand.size();++i) {
                uint64_t k=key(cand[i].node);
                unsigned h=(unsigned)hash64(k)&((1<<20)-1);
                while (all_stamp[h]==generation && all_keys[h]!=k) h=(h+1)&((1<<20)-1);
                if (all_stamp[h]!=generation) {
                    all_stamp[h]=generation; all_keys[h]=k; all_index[h]=unique;
                    cand[unique++]=cand[i];
                } else if (cand[i].node.cost<cand[all_index[h]].node.cost) {
                    cand[all_index[h]]=cand[i];
                }
            }
            cand.resize(unique);
        }
        for (auto &c : cand) c.rank=c.node.cost*100+estimate(c.node,d+1);
        auto better=[](const Candidate &a,const Candidate &b) {
            if (a.rank != b.rank) return a.rank < b.rank;
            if (a.node.cost != b.node.cost) return a.node.cost < b.node.cost;
            return key(a.node) < key(b.node);
        };
        if (deduplicated && cand.size() > (size_t)width) {
            nth_element(cand.begin(),cand.begin()+width,cand.end(),better);
            cand.resize(width);
        }
        sort(cand.begin(),cand.end(),better);
        ++generation;
        auto &next=layers[d+1]; next.reserve(width);
        for (const auto &c : cand) {
            uint64_t k=key(c.node); unsigned h=(unsigned)hash64(k)&((1<<16)-1);
            while (stamp[h]==generation && table[h]!=k) h=(h+1)&((1<<16)-1);
            if (stamp[h]==generation) continue;
            stamp[h]=generation; table[h]=k; next.push_back(c.node);
            if ((int)next.size()==width) break;
        }
    }
    int at=-1, cost=best_cost;
    for (int i=0;i<(int)layers[M].size();++i)
        if (layers[M][i].cost < cost) cost=layers[M][i].cost, at=i;
    if (at<0) return true;
    best_cost=cost; best_path.resize(M);
    for (int d=M;d>0;--d) { best_path[d-1]=layers[d][at]; at=layers[d][at].parent; }
    return true;
}
inline void walkTo(int &x,int &y,int a,int b) {
    while (x>a) { FastIO::pc('U'); FastIO::pc('\n'); --x; }
    while (x<a) { FastIO::pc('D'); FastIO::pc('\n'); ++x; }
    while (y>b) { FastIO::pc('L'); FastIO::pc('\n'); --y; }
    while (y<b) { FastIO::pc('R'); FastIO::pc('\n'); ++y; }
}
int main() {
    using namespace FastIO;
    if (!readInt(N) || !readInt(M) || !readInt(K) || !readInt(T)) return 0;
    readInt(gx[0]); readInt(gy[0]);
    for (int i=1;i<=M;++i) { readInt(gx[i]); readInt(gy[i]); }
    for (int c=0;c<N*N;++c) { coord_x[c]=c/N; coord_y[c]=c%N; }
    for (int i=0;i<M;++i) for (int c=0;c<N;++c) {
        axis_x[i][c]=axisCost(gx[i],gx[i+1],c);
        axis_y[i][c]=axisCost(gy[i],gy[i+1],c);
        for (int e=0;e<N;++e) {
            two_x[i][c*128+e]=axisTwo(gx[i],gx[i+1],c,e);
            two_y[i][c*128+e]=axisTwo(gy[i],gy[i+1],c,e);
        }
    }
    best_cost=T+1;
    // A complete narrow search is retained if a wider search runs out of time.
    for (int width : {96, 512, 2048}) {
        if (elapsed() >= TIME_LIMIT || !runBeam(width)) break;
    }
    direct_moves = true;
    for (int width : {2048, 5120, 8192}) {
        if (elapsed() >= TIME_LIMIT || !runBeam(width)) break;
    }
    // Safe fallback: walking only. Also ensures operation count never exceeds T.
    if (best_cost > T || best_path.size() != (size_t)M) best_path.clear();
    int printed=0;
    for (const Node &z : best_path) if (z.free_circle) {
        writeInt(z.center/N); pc(' '); writeInt(z.center%N); pc('\n'); ++printed;
    }
    while (printed++ < K) { pc('0'); pc(' '); pc('0'); pc('\n'); }
    int x=gx[0], y=gy[0];
    for (int d=0;d<M;++d) {
        if (!best_path.empty() && best_path[d].center!=UNUSED) {
            const Node &z=best_path[d];
            walkTo(x,y,z.dep/N,z.dep%N);
            int cx=z.center/N, cy=z.center%N;
            pc('J'); pc(' '); writeInt(cx); pc(' '); writeInt(cy); pc('\n');
            x=2*cx-x; y=2*cy-y;
            if (z.center2!=UNUSED) {
                walkTo(x,y,z.dep2/N,z.dep2%N);
                cx=z.center2/N; cy=z.center2%N;
                pc('J'); pc(' '); writeInt(cx); pc(' '); writeInt(cy); pc('\n');
                x=2*cx-x; y=2*cy-y;
            }
        }
        walkTo(x,y,gx[d+1],gy[d+1]);
    }
    flush();
}

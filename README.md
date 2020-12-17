# Graph hands-on

### BFS connected
```
import java.util.*;

public class Graph{
  public static ArrayList<Integer> BFS(ArrayList<ArrayList<Integer>> adj, int v, int s){
    boolean visited[] = new boolean[v];
    Queue<Integer> q = new LinkedList<>();
    ArrayList<Integer> al = new ArrayList<>();
    visited[s] = true;
    q.add(s);
    while(!q.isEmpty()){
      int u = q.poll();
      al.add(u);
      for(int n: adj.get(u)){
        if(visited[n]==false){
          q.add(n);
          visited[n] = true;
        }
      }
    }
    return al;
  }
}
```
### BFS disconnected
```
import java.util.*;

public class Graph{
  public static void BFS(ArrayList<ArrayList<Integer>> adj, int s,boolean visited[],ArrayList<Integer> al){
    Queue<Integer> q = new LinkedList<>();
    visited[s] = true;
    q.add(s);
    while(!q.isEmpty()){
      int u = q.poll();
      al.add(u);
      for(int n: adj.get(u)){
        if(visited[n]==false){
          q.add(n);
          visited[n] = true;
        }
      }
    }
  }
  
  public static ArrayList<Integer> BFSDis(ArrayList<ArrayList<Integer>> adj,int v){
    boolean visited[] = new boolean[v];
    ArrayList<Integer> al = new ArrayList<>();
    for(int i=0;i<v;i++){
      if(visited[i]==false){
        BFS(adj,i,visited,al);
      }
    }
    return al;
  }
}
```
### DFS connected
```
import java.util.*;
public class Graph{
  public static void DFSRec(ArrayList<ArrayList<Integer>> adj, int s,boolean visited[],ArrayList<Integer> al){
    visited[s] = true;
    al.add(s);
    for(int v:adj.get(s)){
        if(visited[v]==false){
            DFSRec(adj,v,visited,al);
        }
    }
  }
  
  public static ArrayList<Integer> DFS(ArrayList<ArrayList<Integer>> adj,int v,int s){
    boolean visited[] = new boolean[v];
    ArrayList<Integer> al = new ArrayList<>();
    DFSRec(adj,s,visited,al);
    return al;
  }
}
```
### DFS disconnected
```
import java.util.*;
public class Graph{
  public static void DFSRec(ArrayList<ArrayList<Integer>> adj, int s,boolean visited[],ArrayList<Integer> al){
    visited[s] = true;
    al.add(s);
    for(int v:adj.get(s)){
        if(visited[v]==false){
            DFSRec(adj,v,visited,al);
        }
    }
  }
  
  public static ArrayList<Integer> DFS(ArrayList<ArrayList<Integer>> adj,int v){
    boolean visited[] = new boolean[v];
    ArrayList<Integer> al = new ArrayList<>();
    for(int i=0;i<v;i++){
      if(visited[i]==false){
        DFSRec(adj,i,visited,al);
      }
    }
    return al;
  }
}
```

### Detect cycle using DFS in undirected graph
```
import java.util.*;
public class Graph{
  public static boolean detectCycle(ArrayList<ArrayList<Integer>> adj,int v, int s){
    boolean visited[] = new boolean[v];
    return DFSRecc(adj,s,visited,-1);
  }
  
  public static boolean DFSRecc(ArrayList<ArrayList<Integer>> adj, int s, boolean visited[], int parent){
    visited[s] = true;
    for(int v:adj.get(s)){
      if(visited[v]==false){
          if(DFSRecc(adj,v,visited,s))
            return true;
      } else if(v!=parent)
            return true;
    }
    return false;
  }
}
```
### Detect cycle using BFS in undirected graph
```
import java.util.*;
public class Graph{
  public static boolean detectCycle(ArrayList<ArrayList<Integer>> adj,int v, int s){
    boolean visited[] = new boolean[v];
    int parent[] = new int[v];
    Arrays.fill(parent,-1);
    Queue<Integer> q = new LinkedList<>();
    q.add(s);
    visited[s] = true;
    while(!q.isEmpty()){
        int u = q.poll();
        for(int n:adj.get(u)){
            if(visited[n]==false){
                visited[n] = true;
                q.add(n);
                parent[n] = u;
            } else if(n!=parent[u]){
                    return true;
            }
        }
    }
    return false;
  }
}
```
### Detect cycle using DFS in directed graph
```
import java.util.*;
public class Graph{
   public static boolean DFSRecc(ArrayList<ArrayList<Integer>> adj, int s, boolean visited[], boolean reccSt[]){
      visited[s] = true;
      reccSt[s] = true;
      for(int u:adj.get(s)){
        if(visited[u]==false && DFSRecc(adj,u,visited,reccSt)){
          return true;
        } else if(reccSt[u]==true){
          return true;
        }
      }
      reccSt[s] = false;
      return false;
   }
   
   public static boolean detectCycle(ArrayList<ArrayList<Integer>> adj, int v){
      boolean visited[] = new boolean[v];
      boolean reccSt[] = new boolean[v];
      for(int i=0;i<v;i++){
        if(visited[i]==false){
          if(DFSRecc(ajd,i,visited,reccSt)){
            return true;
          }
        }
      }
      return false;
   }
}
```
### Detect cycle using BFS in directed graph (Using Kahn's algorithm)
```
import java.util.*;
public class Graph{
  public static boolean detectCycle(ArrayList<ArrayList<Integer>> adj, int V){
    int indegree[] = new int[V];
    for(ArrayList<Integer> al: adj){
      for(int v:al){
        indegree[v]++;
      }
    }
    int count = 0;
    Queue<Integer> q = new LinkedList<>();
    for(int i=0;i<V;i++){
      if(indegree[i]==0){
        q.add(i);
      }
    }
    while(!q.isEmpty()){
      int u = q.poll();
      for(int v:adj.get(u)){
        indegree[v]--;
        if(indegree[v]==0){
          q.add(v);
        }
      }
      count++;
    }
    if(count!=V) return true;
    return false;
  }
}
```
### Prim's algorithm (Minimum spanning tree)
* Time complexity is V^2
* Time complexity can be improved to ElogV by implementing heaps and adjecency list
```
import java.util.*;
public class Graph{
    public static int prim(int arr[][], int V){
        int key[] = new int[V];
        Arrays.fill(key,Integer.MAX_VALUE);
        key[0] = 0;
        boolean mSet[] = new boolean[V];
        int res = 0;
        for(int count=0;count<V;count++){
            int u = -1;
            for(int i=0;i<V;i++){
                if(!mSet[i] && (u==-1 || key[i]<key[u])){
                    u = i;
                }
            }
            mSet[u] = true;
            res += key[u];
            for(int v=0;v<V;v++){
                if(!mSet[v] && arr[v][u]!=0 && arr[v][u]<key[v]){
                    key[v] = arr[v][u];
                }
            }
        }
        return res;
    }
}
```

### Articulation points
```
import java.util.*;
public class Graph{
    private static int time = 1;
    public static ArrayList<Integer> articulationPoints(ArrayList<ArrayList<Integer>> graph, int V){
        ArrayList<Integer> pts = new ArrayList<>();
        int d[] = new int[V];
        int l[] = new int[V];
        int parent[] = new int[V];
        for(int i=0;i<V;i++){
            dfs(graph,d,l,i,-1,pts);
        }
        return pts;
    }
    
    public static void dfs(ArrayList<ArrayList<Integer>> graph, int[] d, int[] l, int v, int p,ArrayList<Integer> pts){
        d[v] = time;
        l[v] = time++;
        int child = 0;
        int w = -1;
        for(int u:graph.get(v)){
            if(u==p){
                continue;
            }
            if(d[u]==0){
                child++;
                dfs(graph,d,l,u,v,pts);
                l[v] = Math.min(l[v],l[u]);
                if(d[v]<=l[u]){
                    w = u;
                }
            } else {
                l[v] = Math.min(l[v],d[u]);
            }
        }
        if((p==-1 && child>1) || (p!=-1 && w!=-1)){
            pts.add(v);
        }
    }
}
```

### Bridge edges
```
import java.util.*;
public class Graph{
    private static int time = 1;
    
    public static ArrayList<String> bridgeEdges(ArrayList<ArrayList<Integer>> graph, int V){
        ArrayList<String> pts = new ArrayList<>();
        int d[] = new int[V];
        int l[] = new int[V];
        int parent[] = new int[V];
        for(int i=0;i<V;i++){
            if(d[i]==0){
                dfs(graph,d,l,i,-1,pts);
            }
        }
        return pts;
    }
    
    public static void dfs(ArrayList<ArrayList<Integer>> graph, int[] d, int[] l, int v, int p,ArrayList<String> pts){
        d[v] = time;
        l[v] = time++;
        int w = -1;
        for(int u:graph.get(v)){
            if(u==p){
                continue;
            }
            if(d[u]==0){
                dfs(graph,d,l,u,v,pts);
                l[v] = Math.min(l[v],l[u]);
                if(d[v]<l[u]){  // change from articulation point
                    w = u;
                }
            } else {
                l[v] = Math.min(l[v],d[u]);
            }
        }
        if(w!=-1){  // change from articulation point
            pts.add(v+" -> "+ w);
        }
    }
}
```

### Strongly connected components
* Kosaraju's algorithm
```
import java.util.*;
public class Graph{
    public static ArrayList<ArrayList<Integer>> kosaraju(ArrayList<ArrayList<Integer>> adj, int V){
        Stack<Integer> st = findTopoSort(adj,V);
        ArrayList<ArrayList<Integer>> tr = transpose(adj,V);
        boolean visited[] = new boolean[V];
        ArrayList<ArrayList<Integer>> res = new ArrayList<>();
        while(!st.isEmpty()){
            int u = st.pop();
            if(visited[u]==false){
                ArrayList<Integer> temp = new ArrayList<>();
                DFS(tr,null,temp,visited,u);
                res.add(temp);
            }
        }
        return res;
    }
    
    public static Stack<Integer> findTopoSort(ArrayList<ArrayList<Integer>> adj,int V){
        Stack<Integer> st = new Stack<>();
        boolean visited[] = new boolean[V];
        for(int i=0;i<V;i++){
            if(visited[i]==false){
                DFS(adj,st,null,visited,i);
            }
        }
        return st;
    }
    
    public static void DFS(ArrayList<ArrayList<Integer>> adj, Stack<Integer> st,ArrayList<Integer> al, boolean visited[],int s){
        visited[s] = true;
        if(al!=null){
            al.add(s);
        }
        for(int u:adj.get(s)){
            if(visited[u]==false){
                DFS(adj,st,al,visited,u);
            }
        }
        if(st!=null){
            st.push(s);
        }
    }
    
    public static ArrayList<ArrayList<Integer>> transpose(ArrayList<ArrayList<Integer>> adj,int V){
        ArrayList<ArrayList<Integer>> tr = new ArrayList<>();
        for(int i=0;i<V;i++){
            tr.add(new ArrayList<Integer>());
        }
        for(int i=0;i<V;i++){
            for(int u:adj.get(i)){
                tr.get(u).add(i);
            }
        }
        return tr;
    }
}
```
* Tarjan's algorithm
```
import java.util.*;
public class Graph{

    private int time = 1;
    
    public void find(ArrayList<ArrayList<Integer>> adj, int N)
    {
        int d[] = new int[N];
        int l[] = new int[N];
        boolean reccSt[] = new boolean[N];
        Stack<Integer> st = new Stack<>();
        for(int i=0;i<N;i++){
            if(d[i]==0){
                dfs(adj,d,l,i,reccSt,st);
            }
        }
    }
    
    public void dfs(ArrayList<ArrayList<Integer>> adj,int[] d, int[] l,int v, boolean[] reccSt, Stack<Integer> st){
        d[v] = time;
        l[v] = time++;
        reccSt[v] = true;
        st.push(v);
        for(int u:adj.get(v)){
            if(d[u]==0){
                dfs(adj,d,l,u,reccSt,st);
                l[v] = Math.min(l[v],l[u]);
            } else {
                if(reccSt[u]){
                    l[v] = Math.min(l[v],d[u]);
                }
            }
        }
        int u=-1;
        if(l[v]==d[v]){
            StringBuilder sb = new StringBuilder();
            while(u!=v){
                u = st.pop();
                sb.append(u+" ");
                reccSt[u] = false;
            }
            System.out.print(sb.toString().trim()+",");
        }
    }
}
```

### Shortest distance
* Shortest path in unweighted undirected graph (BFS solution)
  - Time complexity V+E
```
import java.util.*;
public class Graph{
  public static int[] shortestPath(int graph[][],int V, int s){
    int dist[] = new int[V];
    Arrays.fill(dist,Integer.MAX_VALUE);
    dist[s] = 0;
    boolean visited[] = new boolean[V];
    Queue<Integer> q = new LinkedList<>();
    q.add(s);
    visited[s] = true;
    while(!q.isEmpty()){
      int u = q.poll();
      for(int v=0;v<V;v++){
        if(visited[v]==false && graph[u][v]!=0){
          dist[v] = dist[u] + 1;
          visited[v] = true;
          q.add(v);
        }
      }
    }
    return dist;
  }
}
```
* Shortest path in Direct Acyclic Graph (DAG)
  - Time complexity V+E
```
import java.util.*;
public class Graph{
  public static int[] shortestPath(int[][] graph, int V,int s){
    int topo[] = findTopoSort(graph,V);
    int dist[] = new int[V];
    Arrays.fill(dist,Integer.MAX_VALUE);
    dist[s] = 0;
    for(int u:topo){
      for(int v=0;v<V;v++){
        if(graph[u][v]!=0 && dist[v] > dist[u]+graph[u][v]){
          dist[v] = dist[u] + graph[u][v];
        }
      }
    }
    return dist;
  }
  
  public static int[] findTopoSort(int[][] graph, int V){
    int indegree[] = new int[V];
    for(int i=0;i<V;i++){
      for(int j=0;j<V;j++){
        if(graph[i][j]!=0){
          indegree[j]++;
        }
      }
    }
    ArrayList<Integer> al = new ArrayList<>();
    Queue<Integer> q = new LinkedList<>();
    for(int i=0;i<V;i++){
      if(indegree[i]==0){
        q.add(i);
      }
    }
    while(!q.isEmpty()){
      int u = q.poll();
      al.add(u);
      for(int i=0;i<V;i++){
        int v = graph[u][i]
        if(graph[u][i]!=0){
          indegree[v]--;
          if(indegree[v]==0){
            q.add(v);
          }
        }
      }
    }
    return al.toArray();
  }
}
```
* Dijkstra's algorithm (Undirected/Directed, positive weighted, may contain cycles)
  - Time complexity V*V
  - Time complexity can be improved to (V+E)logV if used priority queue and adjecency list
```
import java.util.*;
public class Graph{
  public static int[] dijkstra(int graph[][],int V, int s){
    int dist[] = new int[V];
    Arrays.fill(dist,Integer.MAX_VALUE);
    dist[s] = 0;
    boolean visited[] = new boolean[V];
    for(int count=0;count<V-1;count++){
      int u = -1;
      for(int v=0;v<V;v++){
        if(!visited[v] && (u==-1 || dist[v]<dist[u])){
          u = v;
        }
      }
      visited[u] = true;
      for(int v=0;v<V;v++){
        if(!visited[v] && graph[v][u]!=0){
          dist[v] = Math.min(dist[v],dist[u] + graph[u][v]);
        }
      }
    }
    return dist;
  }
}
```
* Bellman-Ford Algorithm (for negative weights, but also should not contain negative weighted cycle)
  - Time complexity is O(VE)
```
import java.util.*;
public class Graph{
  void BellmanFord(int[][]graph, int V, int src) 
    { 
        int dist[] = new int[V]; 
        Arrays.fill(dist,Integer.MAX_VALUE);
        dist[src] = 0; 
  
        for (int u = 0; u < V-1; u++) { 
            for (int v = 0; v < V; v++) { 
                int weight = graph[u][v];
                if (dist[u] != Integer.MAX_VALUE && weight!=0 && dist[u] + weight < dist[v]) 
                    dist[v] = dist[u] + weight; 
            } 
        } 
        
        for(int u=0;u<V;u++){
          for(int v=0;v<V;v++){
            int weight = graph[u][v];
            if (dist[u] != Integer.MAX_VALUE && weight!=0 && dist[u] + weight < dist[v]){
                System.out.println("Graph contains negative weight cycle"); 
                return; 
            }
          }
        }
        
        for(int i=0;i<V;i++){
          System.out.println(dist[i]);
        }
    } 
}
```

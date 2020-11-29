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
### Detect cycle using BFS in directed graph
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
    for(int d:indegree){
      if(d==0){
        q.add(d);
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

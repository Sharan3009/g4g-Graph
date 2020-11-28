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

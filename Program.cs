using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ILOG.Concert;
using ILOG.CPLEX;

namespace CG_algorithm
{
    class Program
    {
        const int L = 120;//木料最大总长
        const int M = 240;//木料总数
        const int P = 10;//客户人数
        List<int> demands = new List<int>();
        List<double> lengths = new List<double>();
        Cplex origin_model = new Cplex();
        List<INumVar> y;
        INumVar[][] x;
        List<IConstraint> constraints;

        Cplex rmp = new Cplex();//限制主问题
        Cplex sub_problem = new Cplex();//列生成子问题
        List<int[]> m;//限制主问题的约束矩阵
        INumVar[] sub_m;//子问题决策变量
        List<INumVar> rmp_y;//限制主问题的松弛决策变量

        static void Main(string[] args)
        {
            Console.WriteLine("求解原问题...");
            Program pg = new Program();
            pg.InitProperties();
            pg.AddCons();
            pg.SolveModel();
            Console.WriteLine("原问题求解完毕...\n");

            //列生成算法
            pg.CG_Init();
            while (true)
            {
                double sub_obj = pg.Solve_subproblem();
                if (sub_obj > 0)
                {
                    //重构并求解限制主问题
                    pg.Alter_rmp();
                }
                else
                {
                    break;
                }
            }
            Console.ReadKey();
        }

        public void InitProperties()
        {
            demands = new List<int>()
            {
                10,11,11,12,12,12,10,11,12,10
            };
            lengths = new List<double>()
            {
                92,59,97,32,38,55,80,75,108,57
            };

            y = origin_model.NumVarArray(M, 0, 1, GenVarNames(M, "y[{0}]")).ToList();

            x = new INumVar[M][];
            for(int i = 0; i < x.GetLength(0); i++)
            {
                x[i] = origin_model.NumVarArray(P, 0, int.MaxValue, GenVarNames(P, "x[{0}][{1}]", i));
            }

            constraints = new List<IConstraint>();

            m = new List<int[]>();

            sub_m = sub_problem.NumVarArray(lengths.Count, 0, double.MaxValue);
        }

        public void AddCons()
        {
            for (int i = 0; i < P; i++)
            {
                INumExpr len_num = origin_model.NumExpr();
                for (int j = 0; j < M; j++)
                {
                    len_num = origin_model.Sum(len_num, x[j][i]);
                }
                constraints.Add(origin_model.AddGe(len_num, demands[i]));
            }

            for (int i = 0; i < M; i++)
            {
                INumExpr sum_length = origin_model.NumExpr();
                for (int j = 0; j < P; j++)
                {
                    sum_length = origin_model.Sum(sum_length, origin_model.Prod(lengths[j], x[i][j]));
                }
                constraints.Add(origin_model.AddLe(sum_length, origin_model.Prod(L, y[i])));
            }
        }

        public void SolveModel()
        {
            INumExpr obj = origin_model.NumExpr();
            for (int i = 0; i < M; i++)
            {
                obj = origin_model.Sum(obj, y[i]);
            }
            origin_model.AddObjective(ObjectiveSense.Minimize, obj);
            origin_model.ExportModel("model.lp");
            origin_model.Solve();
        }

        public void CG_Init()
        {
            rmp_y = new List<INumVar>();
            //生成初始基可行解，10个基变量，10种切割方案，每个方案只切出某个规格的一个样本，其余舍弃
            rmp_y.AddRange(rmp.NumVarArray(lengths.Count, 0, double.MaxValue, GenVarNames(lengths.Count, "y[{0}]")));
            for(int i = 0; i < lengths.Count; i++)
            {
                int[] temp_m = new int[lengths.Count];
                temp_m[i] = 1;
                m.Add(temp_m);
            }
            //求解限制主问题
            //添加约束
            constraints.Clear();          
            for (int k = 0; k < demands.Count; k++)
            {
                INumExpr rmp_cons = rmp.NumExpr();
                for (int i = 0; i < rmp_y.Count; i++)
                {
                    rmp_cons = rmp.Sum(rmp_cons, rmp.Prod(rmp_y[i], m[i][k]));
                }
                constraints.Add(rmp.AddGe(rmp_cons, demands[k]));
            }
            //目标函数
            INumExpr rmp_obj = rmp.NumExpr();
            for(int i = 0; i < rmp_y.Count; i++)
            {
                rmp_obj = rmp.Sum(rmp_obj, rmp_y[i]);
            }
            rmp.AddObjective(ObjectiveSense.Minimize, rmp_obj);
            rmp.Solve();
            rmp.ExportModel("rmp_initial.lp");
        }

        public double Solve_subproblem()
        {
            //建立子问题
            sub_problem.ClearModel();
            //目标函数
            INumExpr obj = sub_problem.NumExpr();
            for(int i = 0; i < constraints.Count; i++)
            {
                double dual_value = rmp.GetDual((IRange)constraints[i]);
                obj = sub_problem.Sum(obj, sub_problem.Prod(dual_value, sub_m[i]));
            }
            obj = sub_problem.Sum(obj, -1);
            sub_problem.AddObjective(ObjectiveSense.Maximize, obj);
            //添加约束
            ILinearNumExpr cons = sub_problem.LinearNumExpr();
            cons.AddTerms(lengths.ToArray(), sub_m);
            sub_problem.AddLe(cons, L);
            sub_problem.Solve();
            return sub_problem.GetObjValue();
        } 

        public void Alter_rmp()
        {
            Column col = rmp.Column(rmp.GetObjective(), 1);
            for(int i = 0; i < constraints.Count; i++)
            {
                col = col.And(rmp.Column((IRange)constraints[i], sub_problem.GetValue(sub_m[i])));
            }
            INumVar gen_y = rmp.NumVar(col, 0, int.MaxValue);
            rmp_y.Add(gen_y);
            rmp.Solve();
        }
       /// <summary>
        /// generate variables name
        /// </summary>
        /// <param name="length"></param>
        /// <param name="format_name"></param>
        /// <returns></returns>
        public string[] GenVarNames(int length, string format_name)
        {
            string[] names = new string[length];
            for (int i = 0; i < length; i++)
            {
                names[i] = string.Format(format_name, i);
            }
            return names;
        }

        /// <summary>
        /// generate variables name for 2-dimentional variables
        /// </summary>
        /// <param name="length"></param>
        /// <param name="format_name"></param>
        /// <param name="first_dim_idx"></param>
        /// <returns></returns>
        public string[] GenVarNames(int length, string format_name, int first_dim_idx)
        {
            string[] names = new string[length];
            for (int i = 0; i < length; i++)
            {
                names[i] = string.Format(format_name, first_dim_idx, i);
            }
            return names;
        }
    }
}

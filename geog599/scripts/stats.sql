
-- filter within limits and find parameter ranges.
with t as (
	select *, concat(vel_max, vel_min, acc_max, acc_min, residual) as key 
		from trials 
		where vel_max < 5 and vel_min > -5 and acc_max < 9.873 and acc_min > -9.807 and above_max < 10 
		order by filename, above_max, vel_max, abs(vel_min), acc_max, abs(acc_max), residual
), u as (
	select filename, key, count(*) as ct 
	from t 
	group by key, filename
)
select t.filename, count(*) as results, max(alpha) as max_alpha, min(alpha) as min_alpha, max(weight) 
		as max_weight, min(weight) as min_weight, max(smooth) as max_smooth, min(smooth) as min_smooth 
	from t, u 
	where t.key=u.key and ct <= 1 
	group by t.filename;

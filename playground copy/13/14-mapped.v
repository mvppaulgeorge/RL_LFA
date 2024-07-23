// Benchmark "adder" written by ABC on Thu Jul 11 11:50:25 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n324,
    new_n327, new_n329, new_n331, new_n333;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  and002aa1n02x5               g002(.a(\b[3] ), .b(\a[4] ), .o(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\a[3] ), .clkout(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\b[2] ), .clkout(new_n100));
  nanp02aa1n02x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(new_n101), .b(new_n102), .o1(new_n103));
  160nm_ficinv00aa1n08x5       g008(.clk(\a[2] ), .clkout(new_n104));
  160nm_ficinv00aa1n08x5       g009(.clk(\b[1] ), .clkout(new_n105));
  aoi022aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n106));
  aoi012aa1n02x5               g011(.a(new_n106), .b(new_n104), .c(new_n105), .o1(new_n107));
  160nm_ficinv00aa1n08x5       g012(.clk(\a[4] ), .clkout(new_n108));
  aboi22aa1n03x5               g013(.a(\b[3] ), .b(new_n108), .c(new_n99), .d(new_n100), .out0(new_n109));
  oaoi13aa1n02x5               g014(.a(new_n98), .b(new_n109), .c(new_n107), .d(new_n103), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n02x4               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  norp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nano23aa1n02x4               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  norb02aa1n02x5               g025(.a(new_n120), .b(new_n115), .out0(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(new_n111), .clkout(new_n122));
  nanp02aa1n02x5               g027(.a(new_n113), .b(new_n112), .o1(new_n123));
  oai012aa1n02x5               g028(.a(new_n117), .b(new_n118), .c(new_n116), .o1(new_n124));
  oai112aa1n02x5               g029(.a(new_n122), .b(new_n123), .c(new_n115), .d(new_n124), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n110), .d(new_n121), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n97), .out0(new_n128));
  xnrb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanp02aa1n02x5               g034(.a(new_n105), .b(new_n104), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[0] ), .b(\a[1] ), .o1(new_n131));
  aob012aa1n02x5               g036(.a(new_n131), .b(\b[1] ), .c(\a[2] ), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n109), .b(new_n103), .c(new_n132), .d(new_n130), .o1(new_n133));
  nona23aa1n02x4               g038(.a(new_n133), .b(new_n120), .c(new_n115), .d(new_n98), .out0(new_n134));
  norp02aa1n02x5               g039(.a(new_n115), .b(new_n124), .o1(new_n135));
  nano22aa1n02x4               g040(.a(new_n135), .b(new_n122), .c(new_n123), .out0(new_n136));
  norp02aa1n02x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[9] ), .b(\a[10] ), .o1(new_n138));
  oai012aa1n02x5               g043(.a(new_n138), .b(new_n97), .c(new_n137), .o1(new_n139));
  nona23aa1n02x4               g044(.a(new_n126), .b(new_n138), .c(new_n137), .d(new_n97), .out0(new_n140));
  aoai13aa1n02x5               g045(.a(new_n139), .b(new_n140), .c(new_n134), .d(new_n136), .o1(new_n141));
  xorb03aa1n02x5               g046(.a(new_n141), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  aoi012aa1n02x5               g049(.a(new_n143), .b(new_n141), .c(new_n144), .o1(new_n145));
  norp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n146), .clkout(new_n147));
  nanp02aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  xnbna2aa1n03x5               g053(.a(new_n145), .b(new_n148), .c(new_n147), .out0(\s[12] ));
  nona23aa1n02x4               g054(.a(new_n148), .b(new_n144), .c(new_n143), .d(new_n146), .out0(new_n150));
  norp02aa1n02x5               g055(.a(new_n150), .b(new_n140), .o1(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n125), .c(new_n110), .d(new_n121), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(new_n143), .b(new_n148), .o1(new_n153));
  oai112aa1n02x5               g058(.a(new_n153), .b(new_n147), .c(new_n150), .d(new_n139), .o1(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n154), .clkout(new_n155));
  nanp02aa1n02x5               g060(.a(new_n152), .b(new_n155), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1n02x4               g068(.a(new_n163), .b(new_n159), .c(new_n158), .d(new_n162), .out0(new_n164));
  oai012aa1n02x5               g069(.a(new_n163), .b(new_n162), .c(new_n158), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n164), .c(new_n152), .d(new_n155), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  xorc02aa1n02x5               g073(.a(\a[15] ), .b(\b[14] ), .out0(new_n169));
  xorc02aa1n02x5               g074(.a(\a[16] ), .b(\b[15] ), .out0(new_n170));
  aoi112aa1n02x5               g075(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  nano23aa1n02x4               g078(.a(new_n137), .b(new_n97), .c(new_n126), .d(new_n138), .out0(new_n174));
  nano23aa1n02x4               g079(.a(new_n143), .b(new_n146), .c(new_n148), .d(new_n144), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n170), .b(new_n169), .o1(new_n176));
  nano23aa1n02x4               g081(.a(new_n176), .b(new_n164), .c(new_n175), .d(new_n174), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n125), .c(new_n110), .d(new_n121), .o1(new_n178));
  xnrc02aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .out0(new_n179));
  xnrc02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .out0(new_n180));
  norp03aa1n02x5               g085(.a(new_n164), .b(new_n180), .c(new_n179), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  oai022aa1n02x5               g087(.a(new_n176), .b(new_n165), .c(\b[15] ), .d(\a[16] ), .o1(new_n183));
  aoi112aa1n02x5               g088(.a(new_n183), .b(new_n182), .c(new_n154), .d(new_n181), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(new_n184), .b(new_n178), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[17] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\b[16] ), .clkout(new_n188));
  nanp02aa1n02x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  norp02aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  nanb02aa1n02x5               g096(.a(new_n190), .b(new_n191), .out0(new_n192));
  nanp02aa1n02x5               g097(.a(new_n181), .b(new_n151), .o1(new_n193));
  aoi012aa1n02x5               g098(.a(new_n193), .b(new_n134), .c(new_n136), .o1(new_n194));
  norp02aa1n02x5               g099(.a(\b[15] ), .b(\a[16] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(new_n154), .b(new_n181), .o1(new_n196));
  norp03aa1n02x5               g101(.a(new_n180), .b(new_n179), .c(new_n165), .o1(new_n197));
  nona32aa1n02x4               g102(.a(new_n196), .b(new_n197), .c(new_n182), .d(new_n195), .out0(new_n198));
  nanp02aa1n02x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  oai012aa1n02x5               g104(.a(new_n199), .b(new_n198), .c(new_n194), .o1(new_n200));
  xobna2aa1n03x5               g105(.a(new_n192), .b(new_n200), .c(new_n189), .out0(\s[18] ));
  nano22aa1n02x4               g106(.a(new_n192), .b(new_n189), .c(new_n199), .out0(new_n202));
  160nm_ficinv00aa1n08x5       g107(.clk(new_n202), .clkout(new_n203));
  aoai13aa1n02x5               g108(.a(new_n191), .b(new_n190), .c(new_n187), .d(new_n188), .o1(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n203), .c(new_n184), .d(new_n178), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nanb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n210), .clkout(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(\b[19] ), .clkout(new_n212));
  nanb02aa1n02x5               g117(.a(\a[20] ), .b(new_n212), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanp02aa1n02x5               g119(.a(new_n213), .b(new_n214), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  aoi112aa1n02x5               g121(.a(new_n208), .b(new_n216), .c(new_n205), .d(new_n211), .o1(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n208), .clkout(new_n218));
  nanp02aa1n02x5               g123(.a(new_n205), .b(new_n211), .o1(new_n219));
  aoi012aa1n02x5               g124(.a(new_n215), .b(new_n219), .c(new_n218), .o1(new_n220));
  norp02aa1n02x5               g125(.a(new_n220), .b(new_n217), .o1(\s[20] ));
  nona22aa1n02x4               g126(.a(new_n202), .b(new_n210), .c(new_n215), .out0(new_n222));
  nanp02aa1n02x5               g127(.a(new_n208), .b(new_n214), .o1(new_n223));
  norp03aa1n02x5               g128(.a(new_n204), .b(new_n210), .c(new_n215), .o1(new_n224));
  nano22aa1n02x4               g129(.a(new_n224), .b(new_n213), .c(new_n223), .out0(new_n225));
  aoai13aa1n02x5               g130(.a(new_n225), .b(new_n222), .c(new_n184), .d(new_n178), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  xnrc02aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .out0(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(new_n229), .clkout(new_n230));
  xnrc02aa1n02x5               g135(.a(\b[21] ), .b(\a[22] ), .out0(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n231), .clkout(new_n232));
  aoi112aa1n02x5               g137(.a(new_n228), .b(new_n232), .c(new_n226), .d(new_n230), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n228), .clkout(new_n234));
  nanp02aa1n02x5               g139(.a(new_n226), .b(new_n230), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n231), .b(new_n235), .c(new_n234), .o1(new_n236));
  norp02aa1n02x5               g141(.a(new_n236), .b(new_n233), .o1(\s[22] ));
  norp02aa1n02x5               g142(.a(\b[19] ), .b(\a[20] ), .o1(new_n238));
  nona23aa1n02x4               g143(.a(new_n214), .b(new_n209), .c(new_n208), .d(new_n238), .out0(new_n239));
  norp02aa1n02x5               g144(.a(new_n231), .b(new_n229), .o1(new_n240));
  nanb03aa1n02x5               g145(.a(new_n239), .b(new_n240), .c(new_n202), .out0(new_n241));
  oai112aa1n02x5               g146(.a(new_n223), .b(new_n213), .c(new_n239), .d(new_n204), .o1(new_n242));
  oaoi03aa1n02x5               g147(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .o1(new_n243));
  aoi012aa1n02x5               g148(.a(new_n243), .b(new_n242), .c(new_n240), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n241), .c(new_n184), .d(new_n178), .o1(new_n245));
  xorb03aa1n02x5               g150(.a(new_n245), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[23] ), .b(\b[22] ), .out0(new_n248));
  xorc02aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  aoi112aa1n02x5               g154(.a(new_n247), .b(new_n249), .c(new_n245), .d(new_n248), .o1(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n247), .clkout(new_n251));
  nanp02aa1n02x5               g156(.a(new_n245), .b(new_n248), .o1(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n249), .clkout(new_n253));
  aoi012aa1n02x5               g158(.a(new_n253), .b(new_n252), .c(new_n251), .o1(new_n254));
  norp02aa1n02x5               g159(.a(new_n254), .b(new_n250), .o1(\s[24] ));
  nanp02aa1n02x5               g160(.a(new_n249), .b(new_n248), .o1(new_n256));
  nona23aa1n02x4               g161(.a(new_n240), .b(new_n202), .c(new_n256), .d(new_n239), .out0(new_n257));
  xnrc02aa1n02x5               g162(.a(\b[22] ), .b(\a[23] ), .out0(new_n258));
  norb02aa1n02x5               g163(.a(new_n249), .b(new_n258), .out0(new_n259));
  norp02aa1n02x5               g164(.a(\b[23] ), .b(\a[24] ), .o1(new_n260));
  aoi112aa1n02x5               g165(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n261));
  nanp03aa1n02x5               g166(.a(new_n243), .b(new_n248), .c(new_n249), .o1(new_n262));
  nona22aa1n02x4               g167(.a(new_n262), .b(new_n261), .c(new_n260), .out0(new_n263));
  aoi013aa1n02x4               g168(.a(new_n263), .b(new_n242), .c(new_n240), .d(new_n259), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n264), .b(new_n257), .c(new_n184), .d(new_n178), .o1(new_n265));
  xorb03aa1n02x5               g170(.a(new_n265), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g171(.a(\b[24] ), .b(\a[25] ), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[25] ), .b(\b[24] ), .out0(new_n268));
  xorc02aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .out0(new_n269));
  aoi112aa1n02x5               g174(.a(new_n267), .b(new_n269), .c(new_n265), .d(new_n268), .o1(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n267), .clkout(new_n271));
  nanp02aa1n02x5               g176(.a(new_n265), .b(new_n268), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n269), .clkout(new_n273));
  aoi012aa1n02x5               g178(.a(new_n273), .b(new_n272), .c(new_n271), .o1(new_n274));
  norp02aa1n02x5               g179(.a(new_n274), .b(new_n270), .o1(\s[26] ));
  and002aa1n02x5               g180(.a(new_n269), .b(new_n268), .o(new_n276));
  nano22aa1n02x4               g181(.a(new_n241), .b(new_n276), .c(new_n259), .out0(new_n277));
  oai012aa1n02x5               g182(.a(new_n277), .b(new_n198), .c(new_n194), .o1(new_n278));
  nano22aa1n02x4               g183(.a(new_n225), .b(new_n240), .c(new_n259), .out0(new_n279));
  oao003aa1n02x5               g184(.a(\a[26] ), .b(\b[25] ), .c(new_n271), .carry(new_n280));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n280), .clkout(new_n281));
  oaoi13aa1n02x5               g186(.a(new_n281), .b(new_n276), .c(new_n279), .d(new_n263), .o1(new_n282));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  nanp02aa1n02x5               g188(.a(\b[26] ), .b(\a[27] ), .o1(new_n284));
  norb02aa1n02x5               g189(.a(new_n284), .b(new_n283), .out0(new_n285));
  xnbna2aa1n03x5               g190(.a(new_n285), .b(new_n278), .c(new_n282), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n283), .clkout(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[27] ), .b(\a[28] ), .out0(new_n288));
  nona32aa1n02x4               g193(.a(new_n242), .b(new_n256), .c(new_n231), .d(new_n229), .out0(new_n289));
  160nm_ficinv00aa1n08x5       g194(.clk(new_n263), .clkout(new_n290));
  160nm_ficinv00aa1n08x5       g195(.clk(new_n276), .clkout(new_n291));
  aoai13aa1n02x5               g196(.a(new_n280), .b(new_n291), .c(new_n289), .d(new_n290), .o1(new_n292));
  aoai13aa1n02x5               g197(.a(new_n284), .b(new_n292), .c(new_n185), .d(new_n277), .o1(new_n293));
  aoi012aa1n02x5               g198(.a(new_n288), .b(new_n293), .c(new_n287), .o1(new_n294));
  aobi12aa1n02x5               g199(.a(new_n284), .b(new_n278), .c(new_n282), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n287), .c(new_n288), .out0(new_n296));
  norp02aa1n02x5               g201(.a(new_n294), .b(new_n296), .o1(\s[28] ));
  nano22aa1n02x4               g202(.a(new_n288), .b(new_n287), .c(new_n284), .out0(new_n298));
  aoai13aa1n02x5               g203(.a(new_n298), .b(new_n292), .c(new_n185), .d(new_n277), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[28] ), .b(\b[27] ), .c(new_n287), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[28] ), .b(\a[29] ), .out0(new_n301));
  aoi012aa1n02x5               g206(.a(new_n301), .b(new_n299), .c(new_n300), .o1(new_n302));
  aobi12aa1n02x5               g207(.a(new_n298), .b(new_n278), .c(new_n282), .out0(new_n303));
  nano22aa1n02x4               g208(.a(new_n303), .b(new_n300), .c(new_n301), .out0(new_n304));
  norp02aa1n02x5               g209(.a(new_n302), .b(new_n304), .o1(\s[29] ));
  xorb03aa1n02x5               g210(.a(new_n131), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g211(.a(new_n285), .b(new_n301), .c(new_n288), .out0(new_n307));
  aoai13aa1n02x5               g212(.a(new_n307), .b(new_n292), .c(new_n185), .d(new_n277), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .carry(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[29] ), .b(\a[30] ), .out0(new_n310));
  aoi012aa1n02x5               g215(.a(new_n310), .b(new_n308), .c(new_n309), .o1(new_n311));
  aobi12aa1n02x5               g216(.a(new_n307), .b(new_n278), .c(new_n282), .out0(new_n312));
  nano22aa1n02x4               g217(.a(new_n312), .b(new_n309), .c(new_n310), .out0(new_n313));
  norp02aa1n02x5               g218(.a(new_n311), .b(new_n313), .o1(\s[30] ));
  xnrc02aa1n02x5               g219(.a(\b[30] ), .b(\a[31] ), .out0(new_n315));
  norb03aa1n02x5               g220(.a(new_n298), .b(new_n310), .c(new_n301), .out0(new_n316));
  aobi12aa1n02x5               g221(.a(new_n316), .b(new_n278), .c(new_n282), .out0(new_n317));
  oao003aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .c(new_n309), .carry(new_n318));
  nano22aa1n02x4               g223(.a(new_n317), .b(new_n315), .c(new_n318), .out0(new_n319));
  aoai13aa1n02x5               g224(.a(new_n316), .b(new_n292), .c(new_n185), .d(new_n277), .o1(new_n320));
  aoi012aa1n02x5               g225(.a(new_n315), .b(new_n320), .c(new_n318), .o1(new_n321));
  norp02aa1n02x5               g226(.a(new_n321), .b(new_n319), .o1(\s[31] ));
  xobna2aa1n03x5               g227(.a(new_n103), .b(new_n132), .c(new_n130), .out0(\s[3] ));
  aoai13aa1n02x5               g228(.a(new_n101), .b(new_n103), .c(new_n132), .d(new_n130), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g230(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g231(.a(new_n119), .b(new_n110), .c(new_n118), .o1(new_n327));
  xnrb03aa1n02x5               g232(.a(new_n327), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aob012aa1n02x5               g233(.a(new_n124), .b(new_n110), .c(new_n120), .out0(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g235(.a(new_n113), .b(new_n329), .c(new_n114), .o1(new_n331));
  xnbna2aa1n03x5               g236(.a(new_n331), .b(new_n122), .c(new_n112), .out0(\s[8] ));
  norb02aa1n02x5               g237(.a(new_n126), .b(new_n97), .out0(new_n333));
  xnbna2aa1n03x5               g238(.a(new_n333), .b(new_n134), .c(new_n136), .out0(\s[9] ));
endmodule



// Benchmark "adder" written by ABC on Thu Jul 11 12:19:59 2024

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
    new_n125, new_n126, new_n128, new_n130, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n313, new_n315, new_n317, new_n319;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[9] ), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\b[8] ), .clkout(new_n99));
  norp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano23aa1n02x4               g008(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n104));
  nanp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  oa0012aa1n02x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o(new_n108));
  oa0012aa1n02x5               g013(.a(new_n101), .b(new_n102), .c(new_n100), .o(new_n109));
  aoi012aa1n02x5               g014(.a(new_n109), .b(new_n104), .c(new_n108), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  160nm_ficinv00aa1n08x5       g016(.clk(new_n111), .clkout(new_n112));
  oao003aa1n02x5               g017(.a(\a[4] ), .b(\b[3] ), .c(new_n112), .carry(new_n113));
  xorc02aa1n02x5               g018(.a(\a[4] ), .b(\b[3] ), .out0(new_n114));
  nanp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  norb02aa1n02x5               g020(.a(new_n115), .b(new_n111), .out0(new_n116));
  and002aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o(new_n117));
  nanp02aa1n02x5               g022(.a(\b[0] ), .b(\a[1] ), .o1(new_n118));
  norp02aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  oab012aa1n02x4               g024(.a(new_n117), .b(new_n119), .c(new_n118), .out0(new_n120));
  nanp03aa1n02x5               g025(.a(new_n120), .b(new_n114), .c(new_n116), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  nano23aa1n02x4               g027(.a(new_n107), .b(new_n106), .c(new_n122), .d(new_n105), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(new_n123), .b(new_n104), .o1(new_n124));
  aoai13aa1n02x5               g029(.a(new_n110), .b(new_n124), .c(new_n121), .d(new_n113), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(new_n98), .b(new_n99), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  oaoi03aa1n02x5               g032(.a(\a[10] ), .b(\b[9] ), .c(new_n126), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanb02aa1n02x5               g036(.a(new_n130), .b(new_n131), .out0(new_n132));
  160nm_ficinv00aa1n08x5       g037(.clk(new_n132), .clkout(new_n133));
  norp02aa1n02x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n130), .c(new_n128), .d(new_n133), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[9] ), .b(\a[10] ), .o1(new_n138));
  oaib12aa1n02x5               g043(.a(new_n126), .b(\b[9] ), .c(new_n97), .out0(new_n139));
  nanp03aa1n02x5               g044(.a(new_n139), .b(new_n138), .c(new_n133), .o1(new_n140));
  nona22aa1n02x4               g045(.a(new_n140), .b(new_n136), .c(new_n130), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n137), .b(new_n141), .o1(\s[12] ));
  norp02aa1n02x5               g047(.a(\b[9] ), .b(\a[10] ), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n138), .b(new_n143), .c(new_n98), .d(new_n99), .o1(new_n144));
  oa0012aa1n02x5               g049(.a(new_n135), .b(new_n134), .c(new_n130), .o(new_n145));
  160nm_ficinv00aa1n08x5       g050(.clk(new_n145), .clkout(new_n146));
  oai013aa1n02x4               g051(.a(new_n146), .b(new_n144), .c(new_n132), .d(new_n136), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n138), .b(new_n143), .out0(new_n148));
  nano23aa1n02x4               g053(.a(new_n130), .b(new_n134), .c(new_n135), .d(new_n131), .out0(new_n149));
  xorc02aa1n02x5               g054(.a(\a[9] ), .b(\b[8] ), .out0(new_n150));
  nanp03aa1n02x5               g055(.a(new_n149), .b(new_n148), .c(new_n150), .o1(new_n151));
  160nm_ficinv00aa1n08x5       g056(.clk(new_n151), .clkout(new_n152));
  xnrc02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .out0(new_n153));
  160nm_ficinv00aa1n08x5       g058(.clk(new_n153), .clkout(new_n154));
  aoai13aa1n02x5               g059(.a(new_n154), .b(new_n147), .c(new_n125), .d(new_n152), .o1(new_n155));
  aoi112aa1n02x5               g060(.a(new_n147), .b(new_n154), .c(new_n125), .d(new_n152), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(\s[13] ));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  xnrc02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .out0(new_n160));
  xobna2aa1n03x5               g065(.a(new_n160), .b(new_n155), .c(new_n159), .out0(\s[14] ));
  norp02aa1n02x5               g066(.a(new_n160), .b(new_n153), .o1(new_n162));
  oaoi03aa1n02x5               g067(.a(\a[14] ), .b(\b[13] ), .c(new_n159), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n163), .b(new_n147), .c(new_n162), .o1(new_n164));
  nona32aa1n02x4               g069(.a(new_n125), .b(new_n160), .c(new_n153), .d(new_n151), .out0(new_n165));
  xnrc02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .out0(new_n166));
  xobna2aa1n03x5               g071(.a(new_n166), .b(new_n165), .c(new_n164), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  160nm_ficinv00aa1n08x5       g073(.clk(new_n168), .clkout(new_n169));
  aoai13aa1n02x5               g074(.a(new_n169), .b(new_n166), .c(new_n165), .d(new_n164), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  160nm_ficinv00aa1n08x5       g076(.clk(\a[17] ), .clkout(new_n172));
  xnrc02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .out0(new_n173));
  norp02aa1n02x5               g078(.a(new_n173), .b(new_n166), .o1(new_n174));
  nano22aa1n02x4               g079(.a(new_n151), .b(new_n162), .c(new_n174), .out0(new_n175));
  oaoi03aa1n02x5               g080(.a(\a[16] ), .b(\b[15] ), .c(new_n169), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(new_n99), .b(new_n98), .o1(new_n177));
  oaoi03aa1n02x5               g082(.a(\a[10] ), .b(\b[9] ), .c(new_n177), .o1(new_n178));
  aoai13aa1n02x5               g083(.a(new_n162), .b(new_n145), .c(new_n149), .d(new_n178), .o1(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n163), .clkout(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(new_n174), .clkout(new_n181));
  aoi012aa1n02x5               g086(.a(new_n181), .b(new_n179), .c(new_n180), .o1(new_n182));
  aoi112aa1n02x5               g087(.a(new_n182), .b(new_n176), .c(new_n125), .d(new_n175), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(new_n172), .out0(\s[17] ));
  oaoi03aa1n02x5               g089(.a(\a[17] ), .b(\b[16] ), .c(new_n183), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[18] ), .clkout(new_n187));
  xroi22aa1d04x5               g092(.a(new_n172), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(new_n188), .clkout(new_n189));
  oai022aa1n02x5               g094(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n190));
  oaib12aa1n02x5               g095(.a(new_n190), .b(new_n187), .c(\b[17] ), .out0(new_n191));
  oai012aa1n02x5               g096(.a(new_n191), .b(new_n183), .c(new_n189), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g098(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(new_n197), .clkout(new_n198));
  norp02aa1n02x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  aoai13aa1n02x5               g106(.a(new_n201), .b(new_n195), .c(new_n192), .d(new_n198), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(new_n125), .b(new_n175), .o1(new_n203));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n176), .clkout(new_n204));
  oai112aa1n02x5               g109(.a(new_n203), .b(new_n204), .c(new_n164), .d(new_n181), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(\b[16] ), .b(new_n172), .out0(new_n206));
  oaoi03aa1n02x5               g111(.a(\a[18] ), .b(\b[17] ), .c(new_n206), .o1(new_n207));
  aoai13aa1n02x5               g112(.a(new_n198), .b(new_n207), .c(new_n205), .d(new_n188), .o1(new_n208));
  nona22aa1n02x4               g113(.a(new_n208), .b(new_n201), .c(new_n195), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n202), .b(new_n209), .o1(\s[20] ));
  nona23aa1n02x4               g115(.a(new_n200), .b(new_n196), .c(new_n195), .d(new_n199), .out0(new_n211));
  oa0012aa1n02x5               g116(.a(new_n200), .b(new_n199), .c(new_n195), .o(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  oai012aa1n02x5               g118(.a(new_n213), .b(new_n211), .c(new_n191), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  nano23aa1n02x4               g120(.a(new_n195), .b(new_n199), .c(new_n200), .d(new_n196), .out0(new_n216));
  nanp02aa1n02x5               g121(.a(new_n188), .b(new_n216), .o1(new_n217));
  oai012aa1n02x5               g122(.a(new_n215), .b(new_n183), .c(new_n217), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xnrc02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .out0(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(new_n221), .clkout(new_n222));
  xnrc02aa1n02x5               g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n220), .c(new_n218), .d(new_n222), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n217), .clkout(new_n225));
  aoai13aa1n02x5               g130(.a(new_n222), .b(new_n214), .c(new_n205), .d(new_n225), .o1(new_n226));
  nona22aa1n02x4               g131(.a(new_n226), .b(new_n223), .c(new_n220), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(new_n224), .b(new_n227), .o1(\s[22] ));
  norp02aa1n02x5               g133(.a(new_n223), .b(new_n221), .o1(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[22] ), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\b[21] ), .clkout(new_n231));
  oao003aa1n02x5               g136(.a(new_n230), .b(new_n231), .c(new_n220), .carry(new_n232));
  aoi012aa1n02x5               g137(.a(new_n232), .b(new_n214), .c(new_n229), .o1(new_n233));
  nano22aa1n02x4               g138(.a(new_n189), .b(new_n229), .c(new_n216), .out0(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n234), .clkout(new_n235));
  oai012aa1n02x5               g140(.a(new_n233), .b(new_n183), .c(new_n235), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  xorc02aa1n02x5               g143(.a(\a[23] ), .b(\b[22] ), .out0(new_n239));
  xnrc02aa1n02x5               g144(.a(\b[23] ), .b(\a[24] ), .out0(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n238), .c(new_n236), .d(new_n239), .o1(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n233), .clkout(new_n242));
  aoai13aa1n02x5               g147(.a(new_n239), .b(new_n242), .c(new_n205), .d(new_n234), .o1(new_n243));
  nona22aa1n02x4               g148(.a(new_n243), .b(new_n240), .c(new_n238), .out0(new_n244));
  nanp02aa1n02x5               g149(.a(new_n241), .b(new_n244), .o1(\s[24] ));
  norb02aa1n02x5               g150(.a(new_n239), .b(new_n240), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n246), .clkout(new_n247));
  nano32aa1n02x4               g152(.a(new_n247), .b(new_n188), .c(new_n229), .d(new_n216), .out0(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n248), .clkout(new_n249));
  aoai13aa1n02x5               g154(.a(new_n229), .b(new_n212), .c(new_n216), .d(new_n207), .o1(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n232), .clkout(new_n251));
  oai022aa1n02x5               g156(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n252));
  aob012aa1n02x5               g157(.a(new_n252), .b(\b[23] ), .c(\a[24] ), .out0(new_n253));
  aoai13aa1n02x5               g158(.a(new_n253), .b(new_n247), .c(new_n250), .d(new_n251), .o1(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n254), .clkout(new_n255));
  oai012aa1n02x5               g160(.a(new_n255), .b(new_n183), .c(new_n249), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[25] ), .b(\b[24] ), .out0(new_n259));
  xnrc02aa1n02x5               g164(.a(\b[25] ), .b(\a[26] ), .out0(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n258), .c(new_n256), .d(new_n259), .o1(new_n261));
  aoai13aa1n02x5               g166(.a(new_n259), .b(new_n254), .c(new_n205), .d(new_n248), .o1(new_n262));
  nona22aa1n02x4               g167(.a(new_n262), .b(new_n260), .c(new_n258), .out0(new_n263));
  nanp02aa1n02x5               g168(.a(new_n261), .b(new_n263), .o1(\s[26] ));
  nanp02aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .o1(new_n265));
  norb02aa1n02x5               g170(.a(new_n259), .b(new_n260), .out0(new_n266));
  oai022aa1n02x5               g171(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n267));
  aoi022aa1n02x5               g172(.a(new_n254), .b(new_n266), .c(new_n265), .d(new_n267), .o1(new_n268));
  nano23aa1n02x4               g173(.a(new_n217), .b(new_n247), .c(new_n266), .d(new_n229), .out0(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n269), .clkout(new_n270));
  oai012aa1n02x5               g175(.a(new_n268), .b(new_n183), .c(new_n270), .o1(new_n271));
  xorb03aa1n02x5               g176(.a(new_n271), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  aoai13aa1n02x5               g180(.a(new_n275), .b(new_n273), .c(new_n271), .d(new_n274), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n246), .b(new_n232), .c(new_n214), .d(new_n229), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n266), .clkout(new_n278));
  nanp02aa1n02x5               g183(.a(new_n267), .b(new_n265), .o1(new_n279));
  aoai13aa1n02x5               g184(.a(new_n279), .b(new_n278), .c(new_n277), .d(new_n253), .o1(new_n280));
  aoai13aa1n02x5               g185(.a(new_n274), .b(new_n280), .c(new_n205), .d(new_n269), .o1(new_n281));
  nona22aa1n02x4               g186(.a(new_n281), .b(new_n275), .c(new_n273), .out0(new_n282));
  nanp02aa1n02x5               g187(.a(new_n276), .b(new_n282), .o1(\s[28] ));
  norb02aa1n02x5               g188(.a(new_n274), .b(new_n275), .out0(new_n284));
  aoai13aa1n02x5               g189(.a(new_n284), .b(new_n280), .c(new_n205), .d(new_n269), .o1(new_n285));
  160nm_ficinv00aa1n08x5       g190(.clk(new_n273), .clkout(new_n286));
  oaoi03aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n286), .o1(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  nona22aa1n02x4               g193(.a(new_n285), .b(new_n287), .c(new_n288), .out0(new_n289));
  aoai13aa1n02x5               g194(.a(new_n288), .b(new_n287), .c(new_n271), .d(new_n284), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(new_n290), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n118), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g197(.a(new_n274), .b(new_n288), .c(new_n275), .out0(new_n293));
  oao003aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .c(new_n286), .carry(new_n294));
  oaoi03aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .o1(new_n295));
  xorc02aa1n02x5               g200(.a(\a[30] ), .b(\b[29] ), .out0(new_n296));
  160nm_ficinv00aa1n08x5       g201(.clk(new_n296), .clkout(new_n297));
  aoai13aa1n02x5               g202(.a(new_n297), .b(new_n295), .c(new_n271), .d(new_n293), .o1(new_n298));
  aoai13aa1n02x5               g203(.a(new_n293), .b(new_n280), .c(new_n205), .d(new_n269), .o1(new_n299));
  nona22aa1n02x4               g204(.a(new_n299), .b(new_n295), .c(new_n297), .out0(new_n300));
  nanp02aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[30] ));
  nanp02aa1n02x5               g206(.a(new_n295), .b(new_n296), .o1(new_n302));
  oai012aa1n02x5               g207(.a(new_n302), .b(\b[29] ), .c(\a[30] ), .o1(new_n303));
  nano23aa1n02x4               g208(.a(new_n288), .b(new_n275), .c(new_n296), .d(new_n274), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n304), .b(new_n280), .c(new_n205), .d(new_n269), .o1(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  nona22aa1n02x4               g211(.a(new_n305), .b(new_n306), .c(new_n303), .out0(new_n307));
  aoai13aa1n02x5               g212(.a(new_n306), .b(new_n303), .c(new_n271), .d(new_n304), .o1(new_n308));
  nanp02aa1n02x5               g213(.a(new_n308), .b(new_n307), .o1(\s[31] ));
  xobna2aa1n03x5               g214(.a(new_n120), .b(new_n115), .c(new_n112), .out0(\s[3] ));
  oai012aa1n02x5               g215(.a(new_n115), .b(new_n120), .c(new_n111), .o1(new_n311));
  xnrb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  nanp02aa1n02x5               g217(.a(new_n121), .b(new_n113), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g219(.a(new_n107), .b(new_n313), .c(new_n122), .o1(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_fiao0012aa1n02p5x5     g221(.a(new_n108), .b(new_n313), .c(new_n123), .o(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g223(.a(new_n102), .b(new_n317), .c(new_n103), .o1(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g225(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule



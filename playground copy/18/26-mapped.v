// Benchmark "adder" written by ABC on Thu Jul 11 12:14:30 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n158, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n173, new_n174, new_n175, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n211, new_n212, new_n213, new_n214, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n348, new_n351, new_n352, new_n353, new_n355,
    new_n356, new_n358;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(new_n100), .clkout(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  oai012aa1n02x5               g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  160nm_ficinv00aa1n08x5       g010(.clk(\a[4] ), .clkout(new_n106));
  160nm_ficinv00aa1n08x5       g011(.clk(\b[3] ), .clkout(new_n107));
  nanp02aa1n02x5               g012(.a(new_n107), .b(new_n106), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(new_n108), .b(new_n109), .o1(new_n110));
  160nm_ficinv00aa1n08x5       g015(.clk(\a[3] ), .clkout(new_n111));
  160nm_ficinv00aa1n08x5       g016(.clk(\b[2] ), .clkout(new_n112));
  nanp02aa1n02x5               g017(.a(new_n112), .b(new_n111), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(new_n113), .b(new_n114), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  oaoi03aa1n02x5               g021(.a(new_n106), .b(new_n107), .c(new_n116), .o1(new_n117));
  oai013aa1n02x4               g022(.a(new_n117), .b(new_n110), .c(new_n105), .d(new_n115), .o1(new_n118));
  norp02aa1n02x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  norp02aa1n02x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[6] ), .b(\a[7] ), .o1(new_n122));
  nona23aa1n02x4               g027(.a(new_n122), .b(new_n120), .c(new_n119), .d(new_n121), .out0(new_n123));
  xnrc02aa1n02x5               g028(.a(\b[5] ), .b(\a[6] ), .out0(new_n124));
  xnrc02aa1n02x5               g029(.a(\b[4] ), .b(\a[5] ), .out0(new_n125));
  norp03aa1n02x5               g030(.a(new_n123), .b(new_n124), .c(new_n125), .o1(new_n126));
  160nm_ficinv00aa1n08x5       g031(.clk(\a[6] ), .clkout(new_n127));
  160nm_ficinv00aa1n08x5       g032(.clk(\b[5] ), .clkout(new_n128));
  norp02aa1n02x5               g033(.a(\b[4] ), .b(\a[5] ), .o1(new_n129));
  oaoi03aa1n02x5               g034(.a(new_n127), .b(new_n128), .c(new_n129), .o1(new_n130));
  160nm_fiao0012aa1n02p5x5     g035(.a(new_n119), .b(new_n121), .c(new_n120), .o(new_n131));
  oabi12aa1n02x5               g036(.a(new_n131), .b(new_n123), .c(new_n130), .out0(new_n132));
  xorc02aa1n02x5               g037(.a(\a[9] ), .b(\b[8] ), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n132), .c(new_n118), .d(new_n126), .o1(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n99), .b(new_n134), .c(new_n101), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n97), .clkout(new_n136));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n98), .clkout(new_n137));
  aoai13aa1n02x5               g042(.a(new_n136), .b(new_n137), .c(new_n134), .d(new_n101), .o1(new_n138));
  xorb03aa1n02x5               g043(.a(new_n138), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  norp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  aoi112aa1n02x5               g050(.a(new_n145), .b(new_n140), .c(new_n138), .d(new_n142), .o1(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n140), .clkout(new_n147));
  norp03aa1n02x5               g052(.a(new_n105), .b(new_n110), .c(new_n115), .o1(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n117), .clkout(new_n149));
  oai012aa1n02x5               g054(.a(new_n126), .b(new_n148), .c(new_n149), .o1(new_n150));
  nano23aa1n02x4               g055(.a(new_n119), .b(new_n121), .c(new_n122), .d(new_n120), .out0(new_n151));
  oao003aa1n02x5               g056(.a(new_n127), .b(new_n128), .c(new_n129), .carry(new_n152));
  aoi012aa1n02x5               g057(.a(new_n131), .b(new_n151), .c(new_n152), .o1(new_n153));
  160nm_ficinv00aa1n08x5       g058(.clk(new_n133), .clkout(new_n154));
  aoai13aa1n02x5               g059(.a(new_n101), .b(new_n154), .c(new_n150), .d(new_n153), .o1(new_n155));
  aoai13aa1n02x5               g060(.a(new_n142), .b(new_n97), .c(new_n155), .d(new_n98), .o1(new_n156));
  160nm_ficinv00aa1n08x5       g061(.clk(new_n145), .clkout(new_n157));
  aoi012aa1n02x5               g062(.a(new_n157), .b(new_n156), .c(new_n147), .o1(new_n158));
  norp02aa1n02x5               g063(.a(new_n158), .b(new_n146), .o1(\s[12] ));
  nona23aa1n02x4               g064(.a(new_n144), .b(new_n141), .c(new_n140), .d(new_n143), .out0(new_n160));
  nano22aa1n02x4               g065(.a(new_n160), .b(new_n133), .c(new_n99), .out0(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n132), .c(new_n118), .d(new_n126), .o1(new_n162));
  nano23aa1n02x4               g067(.a(new_n140), .b(new_n143), .c(new_n144), .d(new_n141), .out0(new_n163));
  oai012aa1n02x5               g068(.a(new_n98), .b(new_n100), .c(new_n97), .o1(new_n164));
  160nm_ficinv00aa1n08x5       g069(.clk(new_n164), .clkout(new_n165));
  aoi012aa1n02x5               g070(.a(new_n143), .b(new_n140), .c(new_n144), .o1(new_n166));
  aobi12aa1n02x5               g071(.a(new_n166), .b(new_n163), .c(new_n165), .out0(new_n167));
  norp02aa1n02x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  160nm_ficinv00aa1n08x5       g075(.clk(new_n170), .clkout(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n162), .c(new_n167), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g077(.clk(new_n161), .clkout(new_n173));
  aoai13aa1n02x5               g078(.a(new_n167), .b(new_n173), .c(new_n150), .d(new_n153), .o1(new_n174));
  aoi012aa1n02x5               g079(.a(new_n168), .b(new_n174), .c(new_n169), .o1(new_n175));
  xnrb03aa1n02x5               g080(.a(new_n175), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  nona23aa1n02x4               g083(.a(new_n178), .b(new_n169), .c(new_n168), .d(new_n177), .out0(new_n179));
  aoi012aa1n02x5               g084(.a(new_n177), .b(new_n168), .c(new_n178), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n179), .c(new_n162), .d(new_n167), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  norb02aa1n02x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  norp02aa1n02x5               g090(.a(\b[15] ), .b(\a[16] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[15] ), .b(\a[16] ), .o1(new_n187));
  norb02aa1n02x5               g092(.a(new_n187), .b(new_n186), .out0(new_n188));
  aoi112aa1n02x5               g093(.a(new_n188), .b(new_n183), .c(new_n181), .d(new_n185), .o1(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n183), .clkout(new_n190));
  nano23aa1n02x4               g095(.a(new_n168), .b(new_n177), .c(new_n178), .d(new_n169), .out0(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(new_n180), .clkout(new_n192));
  aoai13aa1n02x5               g097(.a(new_n185), .b(new_n192), .c(new_n174), .d(new_n191), .o1(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n188), .clkout(new_n194));
  aoi012aa1n02x5               g099(.a(new_n194), .b(new_n193), .c(new_n190), .o1(new_n195));
  norp02aa1n02x5               g100(.a(new_n195), .b(new_n189), .o1(\s[16] ));
  nano23aa1n02x4               g101(.a(new_n183), .b(new_n186), .c(new_n187), .d(new_n184), .out0(new_n197));
  nanp02aa1n02x5               g102(.a(new_n197), .b(new_n191), .o1(new_n198));
  nano32aa1n02x4               g103(.a(new_n198), .b(new_n163), .c(new_n133), .d(new_n99), .out0(new_n199));
  aoai13aa1n02x5               g104(.a(new_n199), .b(new_n132), .c(new_n118), .d(new_n126), .o1(new_n200));
  oai012aa1n02x5               g105(.a(new_n166), .b(new_n160), .c(new_n164), .o1(new_n201));
  nona23aa1n02x4               g106(.a(new_n187), .b(new_n184), .c(new_n183), .d(new_n186), .out0(new_n202));
  norp02aa1n02x5               g107(.a(new_n202), .b(new_n179), .o1(new_n203));
  aoi012aa1n02x5               g108(.a(new_n186), .b(new_n183), .c(new_n187), .o1(new_n204));
  oai012aa1n02x5               g109(.a(new_n204), .b(new_n202), .c(new_n180), .o1(new_n205));
  aoi012aa1n02x5               g110(.a(new_n205), .b(new_n201), .c(new_n203), .o1(new_n206));
  norp02aa1n02x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(\b[16] ), .b(\a[17] ), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n200), .c(new_n206), .out0(\s[17] ));
  nanp03aa1n02x5               g115(.a(new_n200), .b(new_n206), .c(new_n209), .o1(new_n211));
  norp02aa1n02x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  nanp02aa1n02x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xobna2aa1n03x5               g119(.a(new_n214), .b(new_n211), .c(new_n208), .out0(\s[18] ));
  nano23aa1n02x4               g120(.a(new_n207), .b(new_n212), .c(new_n213), .d(new_n208), .out0(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n216), .clkout(new_n217));
  aoi012aa1n02x5               g122(.a(new_n212), .b(new_n207), .c(new_n213), .o1(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n217), .c(new_n200), .d(new_n206), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[19] ), .b(\b[18] ), .out0(new_n223));
  xorc02aa1n02x5               g128(.a(\a[20] ), .b(\b[19] ), .out0(new_n224));
  aoi112aa1n02x5               g129(.a(new_n222), .b(new_n224), .c(new_n219), .d(new_n223), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n222), .clkout(new_n226));
  nanp02aa1n02x5               g131(.a(new_n161), .b(new_n203), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n206), .b(new_n227), .c(new_n150), .d(new_n153), .o1(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n218), .clkout(new_n229));
  aoai13aa1n02x5               g134(.a(new_n223), .b(new_n229), .c(new_n228), .d(new_n216), .o1(new_n230));
  xnrc02aa1n02x5               g135(.a(\b[19] ), .b(\a[20] ), .out0(new_n231));
  aoi012aa1n02x5               g136(.a(new_n231), .b(new_n230), .c(new_n226), .o1(new_n232));
  norp02aa1n02x5               g137(.a(new_n232), .b(new_n225), .o1(\s[20] ));
  nanp03aa1n02x5               g138(.a(new_n216), .b(new_n223), .c(new_n224), .o1(new_n234));
  xnrc02aa1n02x5               g139(.a(\b[18] ), .b(\a[19] ), .out0(new_n235));
  oao003aa1n02x5               g140(.a(\a[20] ), .b(\b[19] ), .c(new_n226), .carry(new_n236));
  oai013aa1n02x4               g141(.a(new_n236), .b(new_n235), .c(new_n231), .d(new_n218), .o1(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n237), .clkout(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n234), .c(new_n200), .d(new_n206), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  nanp02aa1n02x5               g146(.a(\b[20] ), .b(\a[21] ), .o1(new_n242));
  nanb02aa1n02x5               g147(.a(new_n241), .b(new_n242), .out0(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n243), .clkout(new_n244));
  norp02aa1n02x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  nanp02aa1n02x5               g150(.a(\b[21] ), .b(\a[22] ), .o1(new_n246));
  nanb02aa1n02x5               g151(.a(new_n245), .b(new_n246), .out0(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n247), .clkout(new_n248));
  aoi112aa1n02x5               g153(.a(new_n241), .b(new_n248), .c(new_n239), .d(new_n244), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n241), .clkout(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n234), .clkout(new_n251));
  aoai13aa1n02x5               g156(.a(new_n244), .b(new_n237), .c(new_n228), .d(new_n251), .o1(new_n252));
  aoi012aa1n02x5               g157(.a(new_n247), .b(new_n252), .c(new_n250), .o1(new_n253));
  norp02aa1n02x5               g158(.a(new_n253), .b(new_n249), .o1(\s[22] ));
  nano23aa1n02x4               g159(.a(new_n241), .b(new_n245), .c(new_n246), .d(new_n242), .out0(new_n255));
  nona23aa1n02x4               g160(.a(new_n216), .b(new_n255), .c(new_n231), .d(new_n235), .out0(new_n256));
  aoi012aa1n02x5               g161(.a(new_n245), .b(new_n241), .c(new_n246), .o1(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n257), .clkout(new_n258));
  aoi012aa1n02x5               g163(.a(new_n258), .b(new_n237), .c(new_n255), .o1(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n256), .c(new_n200), .d(new_n206), .o1(new_n260));
  xorb03aa1n02x5               g165(.a(new_n260), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  xorc02aa1n02x5               g167(.a(\a[23] ), .b(\b[22] ), .out0(new_n263));
  xorc02aa1n02x5               g168(.a(\a[24] ), .b(\b[23] ), .out0(new_n264));
  aoi112aa1n02x5               g169(.a(new_n262), .b(new_n264), .c(new_n260), .d(new_n263), .o1(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n262), .clkout(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n256), .clkout(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n259), .clkout(new_n268));
  aoai13aa1n02x5               g173(.a(new_n263), .b(new_n268), .c(new_n228), .d(new_n267), .o1(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[23] ), .b(\a[24] ), .out0(new_n270));
  aoi012aa1n02x5               g175(.a(new_n270), .b(new_n269), .c(new_n266), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n271), .b(new_n265), .o1(\s[24] ));
  nano32aa1n02x4               g177(.a(new_n234), .b(new_n264), .c(new_n255), .d(new_n263), .out0(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n273), .clkout(new_n274));
  nanb03aa1n02x5               g179(.a(new_n218), .b(new_n224), .c(new_n223), .out0(new_n275));
  nanp03aa1n02x5               g180(.a(new_n255), .b(new_n263), .c(new_n264), .o1(new_n276));
  oaoi03aa1n02x5               g181(.a(\a[24] ), .b(\b[23] ), .c(new_n266), .o1(new_n277));
  aoi013aa1n02x4               g182(.a(new_n277), .b(new_n258), .c(new_n263), .d(new_n264), .o1(new_n278));
  aoai13aa1n02x5               g183(.a(new_n278), .b(new_n276), .c(new_n275), .d(new_n236), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n279), .clkout(new_n280));
  aoai13aa1n02x5               g185(.a(new_n280), .b(new_n274), .c(new_n200), .d(new_n206), .o1(new_n281));
  xorb03aa1n02x5               g186(.a(new_n281), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[25] ), .b(\b[24] ), .out0(new_n284));
  xorc02aa1n02x5               g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  aoi112aa1n02x5               g190(.a(new_n283), .b(new_n285), .c(new_n281), .d(new_n284), .o1(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n283), .clkout(new_n287));
  aoai13aa1n02x5               g192(.a(new_n284), .b(new_n279), .c(new_n228), .d(new_n273), .o1(new_n288));
  160nm_ficinv00aa1n08x5       g193(.clk(new_n285), .clkout(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(new_n288), .c(new_n287), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n290), .b(new_n286), .o1(\s[26] ));
  xorc02aa1n02x5               g196(.a(\a[4] ), .b(\b[3] ), .out0(new_n292));
  norb02aa1n02x5               g197(.a(new_n114), .b(new_n116), .out0(new_n293));
  nanb03aa1n02x5               g198(.a(new_n105), .b(new_n292), .c(new_n293), .out0(new_n294));
  nona22aa1n02x4               g199(.a(new_n151), .b(new_n124), .c(new_n125), .out0(new_n295));
  aoai13aa1n02x5               g200(.a(new_n153), .b(new_n295), .c(new_n294), .d(new_n117), .o1(new_n296));
  oabi12aa1n02x5               g201(.a(new_n205), .b(new_n167), .c(new_n198), .out0(new_n297));
  and002aa1n02x5               g202(.a(new_n285), .b(new_n284), .o(new_n298));
  160nm_ficinv00aa1n08x5       g203(.clk(new_n298), .clkout(new_n299));
  nano23aa1n02x4               g204(.a(new_n256), .b(new_n299), .c(new_n263), .d(new_n264), .out0(new_n300));
  aoai13aa1n02x5               g205(.a(new_n300), .b(new_n297), .c(new_n296), .d(new_n199), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .carry(new_n302));
  aobi12aa1n02x5               g207(.a(new_n302), .b(new_n279), .c(new_n298), .out0(new_n303));
  xorc02aa1n02x5               g208(.a(\a[27] ), .b(\b[26] ), .out0(new_n304));
  xnbna2aa1n03x5               g209(.a(new_n304), .b(new_n301), .c(new_n303), .out0(\s[27] ));
  norp02aa1n02x5               g210(.a(\b[26] ), .b(\a[27] ), .o1(new_n306));
  160nm_ficinv00aa1n08x5       g211(.clk(new_n306), .clkout(new_n307));
  nona23aa1n02x4               g212(.a(new_n246), .b(new_n242), .c(new_n241), .d(new_n245), .out0(new_n308));
  norb03aa1n02x5               g213(.a(new_n263), .b(new_n308), .c(new_n270), .out0(new_n309));
  nanp02aa1n02x5               g214(.a(new_n237), .b(new_n309), .o1(new_n310));
  aoai13aa1n02x5               g215(.a(new_n302), .b(new_n299), .c(new_n310), .d(new_n278), .o1(new_n311));
  aoai13aa1n02x5               g216(.a(new_n304), .b(new_n311), .c(new_n228), .d(new_n300), .o1(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[27] ), .b(\a[28] ), .out0(new_n313));
  aoi012aa1n02x5               g218(.a(new_n313), .b(new_n312), .c(new_n307), .o1(new_n314));
  160nm_ficinv00aa1n08x5       g219(.clk(new_n304), .clkout(new_n315));
  aoi012aa1n02x5               g220(.a(new_n315), .b(new_n301), .c(new_n303), .o1(new_n316));
  nano22aa1n02x4               g221(.a(new_n316), .b(new_n307), .c(new_n313), .out0(new_n317));
  norp02aa1n02x5               g222(.a(new_n314), .b(new_n317), .o1(\s[28] ));
  norb02aa1n02x5               g223(.a(new_n304), .b(new_n313), .out0(new_n319));
  aoai13aa1n02x5               g224(.a(new_n319), .b(new_n311), .c(new_n228), .d(new_n300), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[28] ), .b(\b[27] ), .c(new_n307), .carry(new_n321));
  xnrc02aa1n02x5               g226(.a(\b[28] ), .b(\a[29] ), .out0(new_n322));
  aoi012aa1n02x5               g227(.a(new_n322), .b(new_n320), .c(new_n321), .o1(new_n323));
  160nm_ficinv00aa1n08x5       g228(.clk(new_n319), .clkout(new_n324));
  aoi012aa1n02x5               g229(.a(new_n324), .b(new_n301), .c(new_n303), .o1(new_n325));
  nano22aa1n02x4               g230(.a(new_n325), .b(new_n321), .c(new_n322), .out0(new_n326));
  norp02aa1n02x5               g231(.a(new_n323), .b(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g233(.a(new_n304), .b(new_n322), .c(new_n313), .out0(new_n329));
  aoai13aa1n02x5               g234(.a(new_n329), .b(new_n311), .c(new_n228), .d(new_n300), .o1(new_n330));
  oao003aa1n02x5               g235(.a(\a[29] ), .b(\b[28] ), .c(new_n321), .carry(new_n331));
  xnrc02aa1n02x5               g236(.a(\b[29] ), .b(\a[30] ), .out0(new_n332));
  aoi012aa1n02x5               g237(.a(new_n332), .b(new_n330), .c(new_n331), .o1(new_n333));
  160nm_ficinv00aa1n08x5       g238(.clk(new_n329), .clkout(new_n334));
  aoi012aa1n02x5               g239(.a(new_n334), .b(new_n301), .c(new_n303), .o1(new_n335));
  nano22aa1n02x4               g240(.a(new_n335), .b(new_n331), .c(new_n332), .out0(new_n336));
  norp02aa1n02x5               g241(.a(new_n333), .b(new_n336), .o1(\s[30] ));
  nona32aa1n02x4               g242(.a(new_n304), .b(new_n332), .c(new_n322), .d(new_n313), .out0(new_n338));
  aoi012aa1n02x5               g243(.a(new_n338), .b(new_n301), .c(new_n303), .o1(new_n339));
  oao003aa1n02x5               g244(.a(\a[30] ), .b(\b[29] ), .c(new_n331), .carry(new_n340));
  xnrc02aa1n02x5               g245(.a(\b[30] ), .b(\a[31] ), .out0(new_n341));
  nano22aa1n02x4               g246(.a(new_n339), .b(new_n340), .c(new_n341), .out0(new_n342));
  160nm_ficinv00aa1n08x5       g247(.clk(new_n338), .clkout(new_n343));
  aoai13aa1n02x5               g248(.a(new_n343), .b(new_n311), .c(new_n228), .d(new_n300), .o1(new_n344));
  aoi012aa1n02x5               g249(.a(new_n341), .b(new_n344), .c(new_n340), .o1(new_n345));
  norp02aa1n02x5               g250(.a(new_n345), .b(new_n342), .o1(\s[31] ));
  xnbna2aa1n03x5               g251(.a(new_n105), .b(new_n114), .c(new_n113), .out0(\s[3] ));
  oaoi03aa1n02x5               g252(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n348));
  xorb03aa1n02x5               g253(.a(new_n348), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g254(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norp03aa1n02x5               g255(.a(new_n148), .b(new_n149), .c(new_n125), .o1(new_n351));
  aoi112aa1n02x5               g256(.a(new_n351), .b(new_n124), .c(\a[5] ), .d(\b[4] ), .o1(new_n352));
  aoai13aa1n02x5               g257(.a(new_n124), .b(new_n351), .c(\b[4] ), .d(\a[5] ), .o1(new_n353));
  norb02aa1n02x5               g258(.a(new_n353), .b(new_n352), .out0(\s[6] ));
  orn002aa1n02x5               g259(.a(\a[7] ), .b(\b[6] ), .o(new_n355));
  norp02aa1n02x5               g260(.a(new_n352), .b(new_n152), .o1(new_n356));
  xnbna2aa1n03x5               g261(.a(new_n356), .b(new_n355), .c(new_n122), .out0(\s[7] ));
  oaoi13aa1n02x5               g262(.a(new_n121), .b(new_n122), .c(new_n352), .d(new_n152), .o1(new_n358));
  xnrb03aa1n02x5               g263(.a(new_n358), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g264(.a(new_n133), .b(new_n150), .c(new_n153), .out0(\s[9] ));
endmodule



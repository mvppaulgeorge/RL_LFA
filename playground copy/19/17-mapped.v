// Benchmark "adder" written by ABC on Thu Jul 11 12:17:37 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n348, new_n351, new_n352, new_n354, new_n356;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(new_n98), .clkout(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aob012aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  xorc02aa1n02x5               g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  xorc02aa1n02x5               g008(.a(\a[3] ), .b(\b[2] ), .out0(new_n104));
  nanp03aa1n02x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  160nm_ficinv00aa1n08x5       g010(.clk(\a[3] ), .clkout(new_n106));
  160nm_ficinv00aa1n08x5       g011(.clk(\b[2] ), .clkout(new_n107));
  nanp02aa1n02x5               g012(.a(new_n107), .b(new_n106), .o1(new_n108));
  oaoi03aa1n02x5               g013(.a(\a[4] ), .b(\b[3] ), .c(new_n108), .o1(new_n109));
  160nm_ficinv00aa1n08x5       g014(.clk(new_n109), .clkout(new_n110));
  norp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n02x4               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  nona22aa1n02x4               g022(.a(new_n115), .b(new_n116), .c(new_n117), .out0(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\a[6] ), .clkout(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\b[5] ), .clkout(new_n120));
  norp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  oao003aa1n02x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .carry(new_n122));
  160nm_fiao0012aa1n02p5x5     g027(.a(new_n111), .b(new_n113), .c(new_n112), .o(new_n123));
  aoi012aa1n02x5               g028(.a(new_n123), .b(new_n115), .c(new_n122), .o1(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n118), .c(new_n105), .d(new_n110), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoi012aa1n02x5               g031(.a(new_n97), .b(new_n125), .c(new_n126), .o1(new_n127));
  xnrb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  norp02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nano23aa1n02x4               g038(.a(new_n97), .b(new_n132), .c(new_n133), .d(new_n126), .out0(new_n134));
  aoi012aa1n02x5               g039(.a(new_n132), .b(new_n97), .c(new_n133), .o1(new_n135));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n135), .clkout(new_n136));
  aoai13aa1n02x5               g041(.a(new_n131), .b(new_n136), .c(new_n125), .d(new_n134), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n131), .b(new_n136), .c(new_n125), .d(new_n134), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  oai012aa1n02x5               g044(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  aoi012aa1n02x5               g046(.a(new_n98), .b(new_n100), .c(new_n101), .o1(new_n142));
  160nm_ficinv00aa1n08x5       g047(.clk(\a[4] ), .clkout(new_n143));
  nanb02aa1n02x5               g048(.a(\b[3] ), .b(new_n143), .out0(new_n144));
  nanp02aa1n02x5               g049(.a(\b[3] ), .b(\a[4] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(new_n144), .b(new_n145), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(\b[2] ), .b(\a[3] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n108), .b(new_n147), .o1(new_n148));
  norp03aa1n02x5               g053(.a(new_n142), .b(new_n146), .c(new_n148), .o1(new_n149));
  nona23aa1n02x4               g054(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n150));
  norp03aa1n02x5               g055(.a(new_n150), .b(new_n116), .c(new_n117), .o1(new_n151));
  oai012aa1n02x5               g056(.a(new_n151), .b(new_n149), .c(new_n109), .o1(new_n152));
  norp02aa1n02x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  nano23aa1n02x4               g059(.a(new_n129), .b(new_n153), .c(new_n154), .d(new_n130), .out0(new_n155));
  aoi012aa1n02x5               g060(.a(new_n153), .b(new_n129), .c(new_n154), .o1(new_n156));
  aobi12aa1n02x5               g061(.a(new_n156), .b(new_n155), .c(new_n136), .out0(new_n157));
  nanp02aa1n02x5               g062(.a(new_n155), .b(new_n134), .o1(new_n158));
  aoai13aa1n02x5               g063(.a(new_n157), .b(new_n158), .c(new_n152), .d(new_n124), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n161), .b(new_n159), .c(new_n162), .o1(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oai013aa1n02x4               g069(.a(new_n110), .b(new_n142), .c(new_n146), .d(new_n148), .o1(new_n165));
  oaoi03aa1n02x5               g070(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n166));
  oabi12aa1n02x5               g071(.a(new_n123), .b(new_n150), .c(new_n166), .out0(new_n167));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n158), .clkout(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n167), .c(new_n165), .d(new_n151), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nona23aa1n02x4               g076(.a(new_n171), .b(new_n162), .c(new_n161), .d(new_n170), .out0(new_n172));
  aoi012aa1n02x5               g077(.a(new_n170), .b(new_n161), .c(new_n171), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n172), .c(new_n169), .d(new_n157), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nanb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n178), .clkout(new_n179));
  norp02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nanb02aa1n02x5               g086(.a(new_n180), .b(new_n181), .out0(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n182), .clkout(new_n183));
  aoi112aa1n02x5               g088(.a(new_n183), .b(new_n176), .c(new_n174), .d(new_n179), .o1(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(new_n176), .clkout(new_n185));
  nano23aa1n02x4               g090(.a(new_n161), .b(new_n170), .c(new_n171), .d(new_n162), .out0(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(new_n173), .clkout(new_n187));
  aoai13aa1n02x5               g092(.a(new_n179), .b(new_n187), .c(new_n159), .d(new_n186), .o1(new_n188));
  aoi012aa1n02x5               g093(.a(new_n182), .b(new_n188), .c(new_n185), .o1(new_n189));
  norp02aa1n02x5               g094(.a(new_n189), .b(new_n184), .o1(\s[16] ));
  nona22aa1n02x4               g095(.a(new_n186), .b(new_n182), .c(new_n178), .out0(new_n191));
  norp02aa1n02x5               g096(.a(new_n191), .b(new_n158), .o1(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n167), .c(new_n165), .d(new_n151), .o1(new_n193));
  nona23aa1n02x4               g098(.a(new_n154), .b(new_n130), .c(new_n129), .d(new_n153), .out0(new_n194));
  oai012aa1n02x5               g099(.a(new_n156), .b(new_n194), .c(new_n135), .o1(new_n195));
  nona23aa1n02x4               g100(.a(new_n181), .b(new_n177), .c(new_n176), .d(new_n180), .out0(new_n196));
  norp02aa1n02x5               g101(.a(new_n196), .b(new_n172), .o1(new_n197));
  aoi012aa1n02x5               g102(.a(new_n180), .b(new_n176), .c(new_n181), .o1(new_n198));
  oai012aa1n02x5               g103(.a(new_n198), .b(new_n196), .c(new_n173), .o1(new_n199));
  aoi012aa1n02x5               g104(.a(new_n199), .b(new_n195), .c(new_n197), .o1(new_n200));
  xorc02aa1n02x5               g105(.a(\a[17] ), .b(\b[16] ), .out0(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n193), .c(new_n200), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g107(.clk(\a[18] ), .clkout(new_n203));
  nona23aa1n02x4               g108(.a(new_n186), .b(new_n134), .c(new_n196), .d(new_n194), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n200), .b(new_n204), .c(new_n152), .d(new_n124), .o1(new_n205));
  norp02aa1n02x5               g110(.a(\b[16] ), .b(\a[17] ), .o1(new_n206));
  aoi012aa1n02x5               g111(.a(new_n206), .b(new_n205), .c(new_n201), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[17] ), .c(new_n203), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g113(.clk(\a[17] ), .clkout(new_n209));
  xroi22aa1d04x5               g114(.a(new_n209), .b(\b[16] ), .c(new_n203), .d(\b[17] ), .out0(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n210), .clkout(new_n211));
  norp02aa1n02x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  aoi112aa1n02x5               g117(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n213));
  norp02aa1n02x5               g118(.a(new_n213), .b(new_n212), .o1(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n211), .c(new_n193), .d(new_n200), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g121(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  norp02aa1n02x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  aoi112aa1n02x5               g127(.a(new_n218), .b(new_n222), .c(new_n215), .d(new_n219), .o1(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(new_n218), .clkout(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n214), .clkout(new_n225));
  norb02aa1n02x5               g130(.a(new_n219), .b(new_n218), .out0(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n225), .c(new_n205), .d(new_n210), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n222), .clkout(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n227), .c(new_n224), .o1(new_n229));
  norp02aa1n02x5               g134(.a(new_n229), .b(new_n223), .o1(\s[20] ));
  nona23aa1n02x4               g135(.a(new_n221), .b(new_n219), .c(new_n218), .d(new_n220), .out0(new_n231));
  norb02aa1n02x5               g136(.a(new_n210), .b(new_n231), .out0(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  aoi012aa1n02x5               g138(.a(new_n220), .b(new_n218), .c(new_n221), .o1(new_n234));
  oai012aa1n02x5               g139(.a(new_n234), .b(new_n231), .c(new_n214), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n233), .c(new_n193), .d(new_n200), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  xnrc02aa1n02x5               g144(.a(\b[20] ), .b(\a[21] ), .out0(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  norp02aa1n02x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  nanp02aa1n02x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nanb02aa1n02x5               g148(.a(new_n242), .b(new_n243), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  aoi112aa1n02x5               g150(.a(new_n239), .b(new_n245), .c(new_n237), .d(new_n241), .o1(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n239), .clkout(new_n247));
  aoai13aa1n02x5               g152(.a(new_n241), .b(new_n235), .c(new_n205), .d(new_n232), .o1(new_n248));
  aoi012aa1n02x5               g153(.a(new_n244), .b(new_n248), .c(new_n247), .o1(new_n249));
  norp02aa1n02x5               g154(.a(new_n249), .b(new_n246), .o1(\s[22] ));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n231), .clkout(new_n251));
  norp02aa1n02x5               g156(.a(new_n240), .b(new_n244), .o1(new_n252));
  nanp03aa1n02x5               g157(.a(new_n210), .b(new_n251), .c(new_n252), .o1(new_n253));
  aoi012aa1n02x5               g158(.a(new_n242), .b(new_n239), .c(new_n243), .o1(new_n254));
  aobi12aa1n02x5               g159(.a(new_n254), .b(new_n235), .c(new_n252), .out0(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n253), .c(new_n193), .d(new_n200), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(\b[22] ), .b(\a[23] ), .o1(new_n259));
  nanb02aa1n02x5               g164(.a(new_n258), .b(new_n259), .out0(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n260), .clkout(new_n261));
  norp02aa1n02x5               g166(.a(\b[23] ), .b(\a[24] ), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(\b[23] ), .b(\a[24] ), .o1(new_n263));
  nanb02aa1n02x5               g168(.a(new_n262), .b(new_n263), .out0(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n264), .clkout(new_n265));
  aoi112aa1n02x5               g170(.a(new_n258), .b(new_n265), .c(new_n256), .d(new_n261), .o1(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n258), .clkout(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n253), .clkout(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n255), .clkout(new_n269));
  aoai13aa1n02x5               g174(.a(new_n261), .b(new_n269), .c(new_n205), .d(new_n268), .o1(new_n270));
  aoi012aa1n02x5               g175(.a(new_n264), .b(new_n270), .c(new_n267), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n271), .b(new_n266), .o1(\s[24] ));
  nona23aa1n02x4               g177(.a(new_n263), .b(new_n259), .c(new_n258), .d(new_n262), .out0(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n273), .clkout(new_n274));
  nano32aa1n02x4               g179(.a(new_n211), .b(new_n274), .c(new_n251), .d(new_n252), .out0(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n275), .clkout(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n262), .clkout(new_n277));
  nanp02aa1n02x5               g182(.a(new_n258), .b(new_n263), .o1(new_n278));
  oai112aa1n02x5               g183(.a(new_n278), .b(new_n277), .c(new_n273), .d(new_n254), .o1(new_n279));
  aoi013aa1n02x4               g184(.a(new_n279), .b(new_n235), .c(new_n252), .d(new_n274), .o1(new_n280));
  aoai13aa1n02x5               g185(.a(new_n280), .b(new_n276), .c(new_n193), .d(new_n200), .o1(new_n281));
  xorb03aa1n02x5               g186(.a(new_n281), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[25] ), .b(\b[24] ), .out0(new_n284));
  xorc02aa1n02x5               g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  aoi112aa1n02x5               g190(.a(new_n283), .b(new_n285), .c(new_n281), .d(new_n284), .o1(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n283), .clkout(new_n287));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n280), .clkout(new_n288));
  aoai13aa1n02x5               g193(.a(new_n284), .b(new_n288), .c(new_n205), .d(new_n275), .o1(new_n289));
  160nm_ficinv00aa1n08x5       g194(.clk(new_n285), .clkout(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n289), .c(new_n287), .o1(new_n291));
  norp02aa1n02x5               g196(.a(new_n291), .b(new_n286), .o1(\s[26] ));
  oabi12aa1n02x5               g197(.a(new_n199), .b(new_n157), .c(new_n191), .out0(new_n293));
  and002aa1n02x5               g198(.a(new_n285), .b(new_n284), .o(new_n294));
  nano22aa1n02x4               g199(.a(new_n253), .b(new_n294), .c(new_n274), .out0(new_n295));
  aoai13aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n125), .d(new_n192), .o1(new_n296));
  oai112aa1n02x5               g201(.a(new_n226), .b(new_n222), .c(new_n213), .d(new_n212), .o1(new_n297));
  160nm_ficinv00aa1n08x5       g202(.clk(new_n252), .clkout(new_n298));
  aoi112aa1n02x5               g203(.a(new_n298), .b(new_n273), .c(new_n297), .d(new_n234), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .carry(new_n300));
  160nm_ficinv00aa1n08x5       g205(.clk(new_n300), .clkout(new_n301));
  oaoi13aa1n02x5               g206(.a(new_n301), .b(new_n294), .c(new_n299), .d(new_n279), .o1(new_n302));
  xorc02aa1n02x5               g207(.a(\a[27] ), .b(\b[26] ), .out0(new_n303));
  xnbna2aa1n03x5               g208(.a(new_n303), .b(new_n296), .c(new_n302), .out0(\s[27] ));
  norp02aa1n02x5               g209(.a(\b[26] ), .b(\a[27] ), .o1(new_n305));
  160nm_ficinv00aa1n08x5       g210(.clk(new_n305), .clkout(new_n306));
  160nm_ficinv00aa1n08x5       g211(.clk(new_n303), .clkout(new_n307));
  aoi012aa1n02x5               g212(.a(new_n307), .b(new_n296), .c(new_n302), .o1(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[27] ), .b(\a[28] ), .out0(new_n309));
  nano22aa1n02x4               g214(.a(new_n308), .b(new_n306), .c(new_n309), .out0(new_n310));
  nona32aa1n02x4               g215(.a(new_n235), .b(new_n273), .c(new_n244), .d(new_n240), .out0(new_n311));
  norp03aa1n02x5               g216(.a(new_n254), .b(new_n260), .c(new_n264), .o1(new_n312));
  nano22aa1n02x4               g217(.a(new_n312), .b(new_n277), .c(new_n278), .out0(new_n313));
  160nm_ficinv00aa1n08x5       g218(.clk(new_n294), .clkout(new_n314));
  aoai13aa1n02x5               g219(.a(new_n300), .b(new_n314), .c(new_n311), .d(new_n313), .o1(new_n315));
  aoai13aa1n02x5               g220(.a(new_n303), .b(new_n315), .c(new_n205), .d(new_n295), .o1(new_n316));
  aoi012aa1n02x5               g221(.a(new_n309), .b(new_n316), .c(new_n306), .o1(new_n317));
  norp02aa1n02x5               g222(.a(new_n317), .b(new_n310), .o1(\s[28] ));
  norb02aa1n02x5               g223(.a(new_n303), .b(new_n309), .out0(new_n319));
  aoai13aa1n02x5               g224(.a(new_n319), .b(new_n315), .c(new_n205), .d(new_n295), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[28] ), .b(\b[27] ), .c(new_n306), .carry(new_n321));
  xnrc02aa1n02x5               g226(.a(\b[28] ), .b(\a[29] ), .out0(new_n322));
  aoi012aa1n02x5               g227(.a(new_n322), .b(new_n320), .c(new_n321), .o1(new_n323));
  160nm_ficinv00aa1n08x5       g228(.clk(new_n319), .clkout(new_n324));
  aoi012aa1n02x5               g229(.a(new_n324), .b(new_n296), .c(new_n302), .o1(new_n325));
  nano22aa1n02x4               g230(.a(new_n325), .b(new_n321), .c(new_n322), .out0(new_n326));
  norp02aa1n02x5               g231(.a(new_n323), .b(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g233(.a(new_n303), .b(new_n322), .c(new_n309), .out0(new_n329));
  aoai13aa1n02x5               g234(.a(new_n329), .b(new_n315), .c(new_n205), .d(new_n295), .o1(new_n330));
  oao003aa1n02x5               g235(.a(\a[29] ), .b(\b[28] ), .c(new_n321), .carry(new_n331));
  xnrc02aa1n02x5               g236(.a(\b[29] ), .b(\a[30] ), .out0(new_n332));
  aoi012aa1n02x5               g237(.a(new_n332), .b(new_n330), .c(new_n331), .o1(new_n333));
  160nm_ficinv00aa1n08x5       g238(.clk(new_n329), .clkout(new_n334));
  aoi012aa1n02x5               g239(.a(new_n334), .b(new_n296), .c(new_n302), .o1(new_n335));
  nano22aa1n02x4               g240(.a(new_n335), .b(new_n331), .c(new_n332), .out0(new_n336));
  norp02aa1n02x5               g241(.a(new_n333), .b(new_n336), .o1(\s[30] ));
  nona32aa1n02x4               g242(.a(new_n303), .b(new_n332), .c(new_n322), .d(new_n309), .out0(new_n338));
  aoi012aa1n02x5               g243(.a(new_n338), .b(new_n296), .c(new_n302), .o1(new_n339));
  oao003aa1n02x5               g244(.a(\a[30] ), .b(\b[29] ), .c(new_n331), .carry(new_n340));
  xnrc02aa1n02x5               g245(.a(\b[30] ), .b(\a[31] ), .out0(new_n341));
  nano22aa1n02x4               g246(.a(new_n339), .b(new_n340), .c(new_n341), .out0(new_n342));
  160nm_ficinv00aa1n08x5       g247(.clk(new_n338), .clkout(new_n343));
  aoai13aa1n02x5               g248(.a(new_n343), .b(new_n315), .c(new_n205), .d(new_n295), .o1(new_n344));
  aoi012aa1n02x5               g249(.a(new_n341), .b(new_n344), .c(new_n340), .o1(new_n345));
  norp02aa1n02x5               g250(.a(new_n345), .b(new_n342), .o1(\s[31] ));
  xnbna2aa1n03x5               g251(.a(new_n142), .b(new_n147), .c(new_n108), .out0(\s[3] ));
  oaoi03aa1n02x5               g252(.a(\a[3] ), .b(\b[2] ), .c(new_n142), .o1(new_n348));
  xorb03aa1n02x5               g253(.a(new_n348), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g254(.a(new_n165), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g255(.a(\b[4] ), .b(\a[5] ), .o1(new_n351));
  aoi012aa1n02x5               g256(.a(new_n121), .b(new_n165), .c(new_n351), .o1(new_n352));
  xorb03aa1n02x5               g257(.a(new_n352), .b(\b[5] ), .c(new_n119), .out0(\s[6] ));
  oaoi03aa1n02x5               g258(.a(\a[6] ), .b(\b[5] ), .c(new_n352), .o1(new_n354));
  xorb03aa1n02x5               g259(.a(new_n354), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g260(.a(new_n113), .b(new_n354), .c(new_n114), .o1(new_n356));
  xnrb03aa1n02x5               g261(.a(new_n356), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g262(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule



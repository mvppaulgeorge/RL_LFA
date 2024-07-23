// Benchmark "adder" written by ABC on Thu Jul 11 11:11:57 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n196, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n278, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n348, new_n351, new_n353, new_n355;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[9] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[8] ), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(new_n99), .clkout(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aob012aa1n02x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .out0(new_n103));
  norp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n02x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  norp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n02x5               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nanp03aa1n02x5               g014(.a(new_n103), .b(new_n106), .c(new_n109), .o1(new_n110));
  aoi012aa1n02x5               g015(.a(new_n104), .b(new_n107), .c(new_n105), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n02x4               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  xorc02aa1n02x5               g021(.a(\a[6] ), .b(\b[5] ), .out0(new_n117));
  xorc02aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .out0(new_n118));
  nanp03aa1n02x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\a[7] ), .clkout(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\b[6] ), .clkout(new_n121));
  aoai13aa1n02x5               g026(.a(new_n113), .b(new_n112), .c(new_n120), .d(new_n121), .o1(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(\a[5] ), .clkout(new_n123));
  160nm_ficinv00aa1n08x5       g028(.clk(\b[4] ), .clkout(new_n124));
  nanp02aa1n02x5               g029(.a(new_n124), .b(new_n123), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[6] ), .b(\b[5] ), .c(new_n125), .o1(new_n126));
  aobi12aa1n02x5               g031(.a(new_n122), .b(new_n116), .c(new_n126), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n119), .c(new_n110), .d(new_n111), .o1(new_n128));
  oaoi03aa1n02x5               g033(.a(new_n97), .b(new_n98), .c(new_n128), .o1(new_n129));
  xnrb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n131), .c(new_n97), .d(new_n98), .o1(new_n133));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n133), .clkout(new_n134));
  nanp02aa1n02x5               g039(.a(new_n98), .b(new_n97), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[8] ), .b(\a[9] ), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n131), .b(new_n132), .out0(new_n137));
  nano22aa1n02x4               g042(.a(new_n137), .b(new_n135), .c(new_n136), .out0(new_n138));
  aoi012aa1n02x5               g043(.a(new_n134), .b(new_n128), .c(new_n138), .o1(new_n139));
  xnrb03aa1n02x5               g044(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g045(.a(\a[11] ), .b(\b[10] ), .c(new_n139), .o1(new_n141));
  xorb03aa1n02x5               g046(.a(new_n141), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  aoi012aa1n02x5               g047(.a(new_n99), .b(new_n101), .c(new_n102), .o1(new_n143));
  nanb02aa1n02x5               g048(.a(new_n104), .b(new_n105), .out0(new_n144));
  160nm_ficinv00aa1n08x5       g049(.clk(\a[3] ), .clkout(new_n145));
  160nm_ficinv00aa1n08x5       g050(.clk(\b[2] ), .clkout(new_n146));
  nanp02aa1n02x5               g051(.a(new_n146), .b(new_n145), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n147), .b(new_n108), .o1(new_n148));
  norp03aa1n02x5               g053(.a(new_n143), .b(new_n144), .c(new_n148), .o1(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n111), .clkout(new_n150));
  nona23aa1n02x4               g055(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n151));
  xnrc02aa1n02x5               g056(.a(\b[5] ), .b(\a[6] ), .out0(new_n152));
  xnrc02aa1n02x5               g057(.a(\b[4] ), .b(\a[5] ), .out0(new_n153));
  norp03aa1n02x5               g058(.a(new_n151), .b(new_n152), .c(new_n153), .o1(new_n154));
  oai012aa1n02x5               g059(.a(new_n154), .b(new_n149), .c(new_n150), .o1(new_n155));
  norp02aa1n02x5               g060(.a(\b[10] ), .b(\a[11] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[10] ), .b(\a[11] ), .o1(new_n157));
  norp02aa1n02x5               g062(.a(\b[11] ), .b(\a[12] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[11] ), .b(\a[12] ), .o1(new_n159));
  nona23aa1n02x4               g064(.a(new_n159), .b(new_n157), .c(new_n156), .d(new_n158), .out0(new_n160));
  nano23aa1n02x4               g065(.a(new_n160), .b(new_n137), .c(new_n136), .d(new_n135), .out0(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n161), .clkout(new_n162));
  nano23aa1n02x4               g067(.a(new_n156), .b(new_n158), .c(new_n159), .d(new_n157), .out0(new_n163));
  aoi012aa1n02x5               g068(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n164));
  aobi12aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n134), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n162), .c(new_n155), .d(new_n127), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g072(.clk(\a[13] ), .clkout(new_n168));
  160nm_ficinv00aa1n08x5       g073(.clk(\b[12] ), .clkout(new_n169));
  oaoi03aa1n02x5               g074(.a(new_n168), .b(new_n169), .c(new_n166), .o1(new_n170));
  xnrb03aa1n02x5               g075(.a(new_n170), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oai013aa1n02x4               g076(.a(new_n111), .b(new_n143), .c(new_n144), .d(new_n148), .o1(new_n172));
  oaib12aa1n02x5               g077(.a(new_n122), .b(new_n151), .c(new_n126), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n161), .b(new_n173), .c(new_n172), .d(new_n154), .o1(new_n174));
  norp02aa1n02x5               g079(.a(\b[12] ), .b(\a[13] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[12] ), .b(\a[13] ), .o1(new_n176));
  norp02aa1n02x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  nona23aa1n02x4               g083(.a(new_n178), .b(new_n176), .c(new_n175), .d(new_n177), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n178), .b(new_n177), .c(new_n168), .d(new_n169), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n179), .c(new_n174), .d(new_n165), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  nanb02aa1n02x5               g089(.a(new_n183), .b(new_n184), .out0(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(new_n185), .clkout(new_n186));
  norp02aa1n02x5               g091(.a(\b[15] ), .b(\a[16] ), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(\b[15] ), .b(\a[16] ), .o1(new_n188));
  nanb02aa1n02x5               g093(.a(new_n187), .b(new_n188), .out0(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n189), .clkout(new_n190));
  aoi112aa1n02x5               g095(.a(new_n190), .b(new_n183), .c(new_n181), .d(new_n186), .o1(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(new_n183), .clkout(new_n192));
  nano23aa1n02x4               g097(.a(new_n175), .b(new_n177), .c(new_n178), .d(new_n176), .out0(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n180), .clkout(new_n194));
  aoai13aa1n02x5               g099(.a(new_n186), .b(new_n194), .c(new_n166), .d(new_n193), .o1(new_n195));
  aoi012aa1n02x5               g100(.a(new_n189), .b(new_n195), .c(new_n192), .o1(new_n196));
  norp02aa1n02x5               g101(.a(new_n196), .b(new_n191), .o1(\s[16] ));
  nona22aa1n02x4               g102(.a(new_n193), .b(new_n189), .c(new_n185), .out0(new_n198));
  nano22aa1n02x4               g103(.a(new_n198), .b(new_n138), .c(new_n163), .out0(new_n199));
  aoai13aa1n02x5               g104(.a(new_n199), .b(new_n173), .c(new_n172), .d(new_n154), .o1(new_n200));
  oai012aa1n02x5               g105(.a(new_n164), .b(new_n160), .c(new_n133), .o1(new_n201));
  nona23aa1n02x4               g106(.a(new_n188), .b(new_n184), .c(new_n183), .d(new_n187), .out0(new_n202));
  norp02aa1n02x5               g107(.a(new_n202), .b(new_n179), .o1(new_n203));
  aoi012aa1n02x5               g108(.a(new_n187), .b(new_n183), .c(new_n188), .o1(new_n204));
  oai012aa1n02x5               g109(.a(new_n204), .b(new_n202), .c(new_n180), .o1(new_n205));
  aoi012aa1n02x5               g110(.a(new_n205), .b(new_n201), .c(new_n203), .o1(new_n206));
  xorc02aa1n02x5               g111(.a(\a[17] ), .b(\b[16] ), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n200), .c(new_n206), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g113(.clk(\a[17] ), .clkout(new_n209));
  nanb02aa1n02x5               g114(.a(\b[16] ), .b(new_n209), .out0(new_n210));
  oabi12aa1n02x5               g115(.a(new_n205), .b(new_n165), .c(new_n198), .out0(new_n211));
  aoai13aa1n02x5               g116(.a(new_n207), .b(new_n211), .c(new_n128), .d(new_n199), .o1(new_n212));
  xnrc02aa1n02x5               g117(.a(\b[17] ), .b(\a[18] ), .out0(new_n213));
  xobna2aa1n03x5               g118(.a(new_n213), .b(new_n212), .c(new_n210), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g119(.clk(\a[18] ), .clkout(new_n215));
  xroi22aa1d04x5               g120(.a(new_n209), .b(\b[16] ), .c(new_n215), .d(\b[17] ), .out0(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n216), .clkout(new_n217));
  oai022aa1n02x5               g122(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n218));
  oaib12aa1n02x5               g123(.a(new_n218), .b(new_n215), .c(\b[17] ), .out0(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n217), .c(new_n200), .d(new_n206), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(\b[18] ), .b(\a[19] ), .o1(new_n224));
  nanb02aa1n02x5               g129(.a(new_n223), .b(new_n224), .out0(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n225), .clkout(new_n226));
  norp02aa1n02x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(\b[19] ), .b(\a[20] ), .o1(new_n228));
  nanb02aa1n02x5               g133(.a(new_n227), .b(new_n228), .out0(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(new_n229), .clkout(new_n230));
  aoi112aa1n02x5               g135(.a(new_n223), .b(new_n230), .c(new_n220), .d(new_n226), .o1(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n223), .clkout(new_n232));
  nona23aa1n02x4               g137(.a(new_n193), .b(new_n138), .c(new_n202), .d(new_n160), .out0(new_n233));
  aoai13aa1n02x5               g138(.a(new_n206), .b(new_n233), .c(new_n155), .d(new_n127), .o1(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n219), .clkout(new_n235));
  aoai13aa1n02x5               g140(.a(new_n226), .b(new_n235), .c(new_n234), .d(new_n216), .o1(new_n236));
  aoi012aa1n02x5               g141(.a(new_n229), .b(new_n236), .c(new_n232), .o1(new_n237));
  norp02aa1n02x5               g142(.a(new_n237), .b(new_n231), .o1(\s[20] ));
  nona23aa1n02x4               g143(.a(new_n230), .b(new_n207), .c(new_n213), .d(new_n225), .out0(new_n239));
  nona23aa1n02x4               g144(.a(new_n228), .b(new_n224), .c(new_n223), .d(new_n227), .out0(new_n240));
  oaoi03aa1n02x5               g145(.a(\a[20] ), .b(\b[19] ), .c(new_n232), .o1(new_n241));
  oabi12aa1n02x5               g146(.a(new_n241), .b(new_n240), .c(new_n219), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n242), .clkout(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n239), .c(new_n200), .d(new_n206), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g150(.a(\b[20] ), .b(\a[21] ), .o1(new_n246));
  xnrc02aa1n02x5               g151(.a(\b[20] ), .b(\a[21] ), .out0(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n247), .clkout(new_n248));
  norp02aa1n02x5               g153(.a(\b[21] ), .b(\a[22] ), .o1(new_n249));
  nanp02aa1n02x5               g154(.a(\b[21] ), .b(\a[22] ), .o1(new_n250));
  nanb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n251), .clkout(new_n252));
  aoi112aa1n02x5               g157(.a(new_n246), .b(new_n252), .c(new_n244), .d(new_n248), .o1(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n246), .clkout(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n239), .clkout(new_n255));
  aoai13aa1n02x5               g160(.a(new_n248), .b(new_n242), .c(new_n234), .d(new_n255), .o1(new_n256));
  aoi012aa1n02x5               g161(.a(new_n251), .b(new_n256), .c(new_n254), .o1(new_n257));
  norp02aa1n02x5               g162(.a(new_n257), .b(new_n253), .o1(\s[22] ));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n240), .clkout(new_n259));
  norp02aa1n02x5               g164(.a(new_n247), .b(new_n251), .o1(new_n260));
  nanp03aa1n02x5               g165(.a(new_n216), .b(new_n259), .c(new_n260), .o1(new_n261));
  oai012aa1n02x5               g166(.a(new_n250), .b(new_n249), .c(new_n246), .o1(new_n262));
  aobi12aa1n02x5               g167(.a(new_n262), .b(new_n242), .c(new_n260), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n261), .c(new_n200), .d(new_n206), .o1(new_n264));
  xorb03aa1n02x5               g169(.a(new_n264), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  nanp02aa1n02x5               g171(.a(\b[22] ), .b(\a[23] ), .o1(new_n267));
  norb02aa1n02x5               g172(.a(new_n267), .b(new_n266), .out0(new_n268));
  norp02aa1n02x5               g173(.a(\b[23] ), .b(\a[24] ), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(\b[23] ), .b(\a[24] ), .o1(new_n270));
  norb02aa1n02x5               g175(.a(new_n270), .b(new_n269), .out0(new_n271));
  aoi112aa1n02x5               g176(.a(new_n266), .b(new_n271), .c(new_n264), .d(new_n268), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n266), .clkout(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n261), .clkout(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n263), .clkout(new_n275));
  aoai13aa1n02x5               g180(.a(new_n268), .b(new_n275), .c(new_n234), .d(new_n274), .o1(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n271), .clkout(new_n277));
  aoi012aa1n02x5               g182(.a(new_n277), .b(new_n276), .c(new_n273), .o1(new_n278));
  norp02aa1n02x5               g183(.a(new_n278), .b(new_n272), .o1(\s[24] ));
  nona23aa1n02x4               g184(.a(new_n270), .b(new_n267), .c(new_n266), .d(new_n269), .out0(new_n280));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n280), .clkout(new_n281));
  nano22aa1n02x4               g186(.a(new_n239), .b(new_n260), .c(new_n281), .out0(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n282), .clkout(new_n283));
  nona32aa1n02x4               g188(.a(new_n242), .b(new_n280), .c(new_n251), .d(new_n247), .out0(new_n284));
  oaoi03aa1n02x5               g189(.a(\a[24] ), .b(\b[23] ), .c(new_n273), .o1(new_n285));
  oab012aa1n02x4               g190(.a(new_n285), .b(new_n280), .c(new_n262), .out0(new_n286));
  nanp02aa1n02x5               g191(.a(new_n284), .b(new_n286), .o1(new_n287));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n287), .clkout(new_n288));
  aoai13aa1n02x5               g193(.a(new_n288), .b(new_n283), .c(new_n200), .d(new_n206), .o1(new_n289));
  xorb03aa1n02x5               g194(.a(new_n289), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g195(.a(\b[24] ), .b(\a[25] ), .o1(new_n291));
  xorc02aa1n02x5               g196(.a(\a[25] ), .b(\b[24] ), .out0(new_n292));
  xorc02aa1n02x5               g197(.a(\a[26] ), .b(\b[25] ), .out0(new_n293));
  aoi112aa1n02x5               g198(.a(new_n291), .b(new_n293), .c(new_n289), .d(new_n292), .o1(new_n294));
  160nm_ficinv00aa1n08x5       g199(.clk(new_n291), .clkout(new_n295));
  aoai13aa1n02x5               g200(.a(new_n292), .b(new_n287), .c(new_n234), .d(new_n282), .o1(new_n296));
  160nm_ficinv00aa1n08x5       g201(.clk(new_n293), .clkout(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n296), .c(new_n295), .o1(new_n298));
  norp02aa1n02x5               g203(.a(new_n298), .b(new_n294), .o1(\s[26] ));
  and002aa1n02x5               g204(.a(new_n293), .b(new_n292), .o(new_n300));
  nano22aa1n02x4               g205(.a(new_n261), .b(new_n300), .c(new_n281), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n301), .b(new_n211), .c(new_n128), .d(new_n199), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[26] ), .b(\b[25] ), .c(new_n295), .carry(new_n303));
  160nm_ficinv00aa1n08x5       g208(.clk(new_n303), .clkout(new_n304));
  aoi012aa1n02x5               g209(.a(new_n304), .b(new_n287), .c(new_n300), .o1(new_n305));
  xorc02aa1n02x5               g210(.a(\a[27] ), .b(\b[26] ), .out0(new_n306));
  xnbna2aa1n03x5               g211(.a(new_n306), .b(new_n305), .c(new_n302), .out0(\s[27] ));
  norp02aa1n02x5               g212(.a(\b[26] ), .b(\a[27] ), .o1(new_n308));
  160nm_ficinv00aa1n08x5       g213(.clk(new_n308), .clkout(new_n309));
  160nm_ficinv00aa1n08x5       g214(.clk(new_n306), .clkout(new_n310));
  aoi012aa1n02x5               g215(.a(new_n310), .b(new_n305), .c(new_n302), .o1(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[27] ), .b(\a[28] ), .out0(new_n312));
  nano22aa1n02x4               g217(.a(new_n311), .b(new_n309), .c(new_n312), .out0(new_n313));
  160nm_ficinv00aa1n08x5       g218(.clk(new_n300), .clkout(new_n314));
  aoai13aa1n02x5               g219(.a(new_n303), .b(new_n314), .c(new_n284), .d(new_n286), .o1(new_n315));
  aoai13aa1n02x5               g220(.a(new_n306), .b(new_n315), .c(new_n234), .d(new_n301), .o1(new_n316));
  aoi012aa1n02x5               g221(.a(new_n312), .b(new_n316), .c(new_n309), .o1(new_n317));
  norp02aa1n02x5               g222(.a(new_n317), .b(new_n313), .o1(\s[28] ));
  norb02aa1n02x5               g223(.a(new_n306), .b(new_n312), .out0(new_n319));
  aoai13aa1n02x5               g224(.a(new_n319), .b(new_n315), .c(new_n234), .d(new_n301), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[28] ), .b(\b[27] ), .c(new_n309), .carry(new_n321));
  xnrc02aa1n02x5               g226(.a(\b[28] ), .b(\a[29] ), .out0(new_n322));
  aoi012aa1n02x5               g227(.a(new_n322), .b(new_n320), .c(new_n321), .o1(new_n323));
  160nm_ficinv00aa1n08x5       g228(.clk(new_n319), .clkout(new_n324));
  aoi012aa1n02x5               g229(.a(new_n324), .b(new_n305), .c(new_n302), .o1(new_n325));
  nano22aa1n02x4               g230(.a(new_n325), .b(new_n321), .c(new_n322), .out0(new_n326));
  norp02aa1n02x5               g231(.a(new_n323), .b(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g233(.a(new_n306), .b(new_n322), .c(new_n312), .out0(new_n329));
  aoai13aa1n02x5               g234(.a(new_n329), .b(new_n315), .c(new_n234), .d(new_n301), .o1(new_n330));
  oao003aa1n02x5               g235(.a(\a[29] ), .b(\b[28] ), .c(new_n321), .carry(new_n331));
  xnrc02aa1n02x5               g236(.a(\b[29] ), .b(\a[30] ), .out0(new_n332));
  aoi012aa1n02x5               g237(.a(new_n332), .b(new_n330), .c(new_n331), .o1(new_n333));
  160nm_ficinv00aa1n08x5       g238(.clk(new_n329), .clkout(new_n334));
  aoi012aa1n02x5               g239(.a(new_n334), .b(new_n305), .c(new_n302), .o1(new_n335));
  nano22aa1n02x4               g240(.a(new_n335), .b(new_n331), .c(new_n332), .out0(new_n336));
  norp02aa1n02x5               g241(.a(new_n333), .b(new_n336), .o1(\s[30] ));
  xnrc02aa1n02x5               g242(.a(\b[30] ), .b(\a[31] ), .out0(new_n338));
  nona32aa1n02x4               g243(.a(new_n306), .b(new_n332), .c(new_n322), .d(new_n312), .out0(new_n339));
  aoi012aa1n02x5               g244(.a(new_n339), .b(new_n305), .c(new_n302), .o1(new_n340));
  oao003aa1n02x5               g245(.a(\a[30] ), .b(\b[29] ), .c(new_n331), .carry(new_n341));
  nano22aa1n02x4               g246(.a(new_n340), .b(new_n338), .c(new_n341), .out0(new_n342));
  160nm_ficinv00aa1n08x5       g247(.clk(new_n339), .clkout(new_n343));
  aoai13aa1n02x5               g248(.a(new_n343), .b(new_n315), .c(new_n234), .d(new_n301), .o1(new_n344));
  aoi012aa1n02x5               g249(.a(new_n338), .b(new_n344), .c(new_n341), .o1(new_n345));
  norp02aa1n02x5               g250(.a(new_n345), .b(new_n342), .o1(\s[31] ));
  xnbna2aa1n03x5               g251(.a(new_n143), .b(new_n108), .c(new_n147), .out0(\s[3] ));
  oaoi03aa1n02x5               g252(.a(\a[3] ), .b(\b[2] ), .c(new_n143), .o1(new_n348));
  xorb03aa1n02x5               g253(.a(new_n348), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g254(.a(new_n172), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g255(.a(new_n123), .b(new_n124), .c(new_n172), .o1(new_n351));
  xnrc02aa1n02x5               g256(.a(new_n351), .b(new_n117), .out0(\s[6] ));
  oaoi03aa1n02x5               g257(.a(\a[6] ), .b(\b[5] ), .c(new_n351), .o1(new_n353));
  xorb03aa1n02x5               g258(.a(new_n353), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g259(.a(new_n120), .b(new_n121), .c(new_n353), .o1(new_n355));
  xnrb03aa1n02x5               g260(.a(new_n355), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g261(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule



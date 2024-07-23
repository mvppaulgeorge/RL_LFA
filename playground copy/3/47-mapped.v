// Benchmark "adder" written by ABC on Wed Jul 10 16:57:32 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n312, new_n315, new_n316, new_n318, new_n319, new_n320,
    new_n321, new_n322, new_n324, new_n326;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xnrc02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  and002aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(new_n98), .clkout(new_n99));
  and002aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\a[3] ), .clkout(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\b[2] ), .clkout(new_n102));
  nanp02aa1n02x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(new_n103), .b(new_n104), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  oa0022aa1n02x5               g014(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n110));
  oaoi13aa1n02x5               g015(.a(new_n100), .b(new_n110), .c(new_n109), .d(new_n105), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  norp03aa1n02x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n111), .b(new_n119), .o1(new_n120));
  norp02aa1n02x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  norp02aa1n02x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  aoi012aa1n02x5               g028(.a(new_n121), .b(new_n123), .c(new_n122), .o1(new_n124));
  oai012aa1n02x5               g029(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n125));
  oai012aa1n02x5               g030(.a(new_n125), .b(new_n116), .c(new_n124), .o1(new_n126));
  xnrc02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  nona22aa1n02x4               g032(.a(new_n120), .b(new_n126), .c(new_n127), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n97), .b(new_n128), .c(new_n99), .out0(\s[10] ));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  160nm_ficinv00aa1n08x5       g035(.clk(new_n130), .clkout(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  and002aa1n02x5               g038(.a(\b[9] ), .b(\a[10] ), .o(new_n134));
  160nm_ficinv00aa1n08x5       g039(.clk(new_n134), .clkout(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n133), .c(new_n128), .d(new_n99), .o1(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n131), .c(new_n132), .out0(\s[11] ));
  norb02aa1n02x5               g042(.a(new_n132), .b(new_n130), .out0(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n138), .clkout(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  160nm_ficinv00aa1n08x5       g047(.clk(new_n142), .clkout(new_n143));
  oai112aa1n02x5               g048(.a(new_n131), .b(new_n143), .c(new_n136), .d(new_n139), .o1(new_n144));
  oaoi13aa1n02x5               g049(.a(new_n143), .b(new_n131), .c(new_n136), .d(new_n139), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n144), .b(new_n145), .out0(\s[12] ));
  nano23aa1n02x4               g051(.a(new_n130), .b(new_n140), .c(new_n141), .d(new_n132), .out0(new_n147));
  oai022aa1n02x5               g052(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n148));
  nona32aa1n02x4               g053(.a(new_n147), .b(new_n148), .c(new_n98), .d(new_n134), .out0(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n126), .c(new_n111), .d(new_n119), .o1(new_n151));
  oab012aa1n02x4               g056(.a(new_n133), .b(\a[9] ), .c(\b[8] ), .out0(new_n152));
  nona23aa1n02x4               g057(.a(new_n142), .b(new_n138), .c(new_n152), .d(new_n134), .out0(new_n153));
  oai012aa1n02x5               g058(.a(new_n141), .b(new_n140), .c(new_n130), .o1(new_n154));
  and002aa1n02x5               g059(.a(new_n153), .b(new_n154), .o(new_n155));
  nanp02aa1n02x5               g060(.a(new_n151), .b(new_n155), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  norp02aa1n02x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nano23aa1n02x4               g071(.a(new_n158), .b(new_n165), .c(new_n166), .d(new_n159), .out0(new_n167));
  aoi012aa1n02x5               g072(.a(new_n165), .b(new_n158), .c(new_n166), .o1(new_n168));
  160nm_ficinv00aa1n08x5       g073(.clk(new_n168), .clkout(new_n169));
  aoai13aa1n02x5               g074(.a(new_n164), .b(new_n169), .c(new_n156), .d(new_n167), .o1(new_n170));
  aoi112aa1n02x5               g075(.a(new_n164), .b(new_n169), .c(new_n156), .d(new_n167), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(\s[15] ));
  norp02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  nona22aa1n02x4               g080(.a(new_n170), .b(new_n175), .c(new_n162), .out0(new_n176));
  160nm_ficinv00aa1n08x5       g081(.clk(new_n175), .clkout(new_n177));
  oaoi13aa1n02x5               g082(.a(new_n177), .b(new_n170), .c(\a[15] ), .d(\b[14] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n176), .b(new_n178), .out0(\s[16] ));
  nano23aa1n02x4               g084(.a(new_n162), .b(new_n173), .c(new_n174), .d(new_n163), .out0(new_n180));
  nanp02aa1n02x5               g085(.a(new_n180), .b(new_n167), .o1(new_n181));
  norp02aa1n02x5               g086(.a(new_n149), .b(new_n181), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n126), .c(new_n111), .d(new_n119), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n180), .b(new_n169), .o1(new_n184));
  aoi012aa1n02x5               g089(.a(new_n181), .b(new_n153), .c(new_n154), .o1(new_n185));
  oai012aa1n02x5               g090(.a(new_n174), .b(new_n173), .c(new_n162), .o1(new_n186));
  nano22aa1n02x4               g091(.a(new_n185), .b(new_n184), .c(new_n186), .out0(new_n187));
  nanp02aa1n02x5               g092(.a(new_n187), .b(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[18] ), .clkout(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(\a[17] ), .clkout(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(\b[16] ), .clkout(new_n192));
  oaoi03aa1n02x5               g097(.a(new_n191), .b(new_n192), .c(new_n188), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[17] ), .c(new_n190), .out0(\s[18] ));
  xroi22aa1d04x5               g099(.a(new_n191), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n195));
  nanp02aa1n02x5               g100(.a(new_n192), .b(new_n191), .o1(new_n196));
  oaoi03aa1n02x5               g101(.a(\a[18] ), .b(\b[17] ), .c(new_n196), .o1(new_n197));
  norp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  160nm_ficinv00aa1n08x5       g105(.clk(new_n200), .clkout(new_n201));
  aoai13aa1n02x5               g106(.a(new_n201), .b(new_n197), .c(new_n188), .d(new_n195), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n201), .b(new_n197), .c(new_n188), .d(new_n195), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g110(.clk(new_n198), .clkout(new_n206));
  norp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  nanp03aa1n02x5               g114(.a(new_n202), .b(new_n206), .c(new_n209), .o1(new_n210));
  aoi012aa1n02x5               g115(.a(new_n209), .b(new_n202), .c(new_n206), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(\s[20] ));
  nano23aa1n02x4               g117(.a(new_n198), .b(new_n207), .c(new_n208), .d(new_n199), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n195), .b(new_n213), .o1(new_n214));
  oai022aa1n02x5               g119(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n215));
  oaib12aa1n02x5               g120(.a(new_n215), .b(new_n190), .c(\b[17] ), .out0(new_n216));
  nona23aa1n02x4               g121(.a(new_n208), .b(new_n199), .c(new_n198), .d(new_n207), .out0(new_n217));
  oaoi03aa1n02x5               g122(.a(\a[20] ), .b(\b[19] ), .c(new_n206), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  oai012aa1n02x5               g124(.a(new_n219), .b(new_n217), .c(new_n216), .o1(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n220), .clkout(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n214), .c(new_n187), .d(new_n183), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  xorc02aa1n02x5               g129(.a(\a[21] ), .b(\b[20] ), .out0(new_n225));
  xorc02aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .out0(new_n226));
  aoi112aa1n02x5               g131(.a(new_n224), .b(new_n226), .c(new_n222), .d(new_n225), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n226), .b(new_n224), .c(new_n222), .d(new_n225), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[21] ), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\a[22] ), .clkout(new_n231));
  xroi22aa1d04x5               g136(.a(new_n230), .b(\b[20] ), .c(new_n231), .d(\b[21] ), .out0(new_n232));
  nanp03aa1n02x5               g137(.a(new_n232), .b(new_n195), .c(new_n213), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\b[21] ), .clkout(new_n234));
  oao003aa1n02x5               g139(.a(new_n231), .b(new_n234), .c(new_n224), .carry(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n220), .c(new_n232), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n233), .c(new_n187), .d(new_n183), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  xorc02aa1n02x5               g144(.a(\a[23] ), .b(\b[22] ), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  aoi112aa1n02x5               g146(.a(new_n239), .b(new_n241), .c(new_n237), .d(new_n240), .o1(new_n242));
  aoai13aa1n02x5               g147(.a(new_n241), .b(new_n239), .c(new_n237), .d(new_n240), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(\s[24] ));
  and002aa1n02x5               g149(.a(new_n241), .b(new_n240), .o(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n245), .clkout(new_n246));
  nano32aa1n02x4               g151(.a(new_n246), .b(new_n232), .c(new_n195), .d(new_n213), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n232), .b(new_n218), .c(new_n213), .d(new_n197), .o1(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n235), .clkout(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(\a[24] ), .clkout(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(\b[23] ), .clkout(new_n251));
  oao003aa1n02x5               g156(.a(new_n250), .b(new_n251), .c(new_n239), .carry(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n252), .clkout(new_n253));
  aoai13aa1n02x5               g158(.a(new_n253), .b(new_n246), .c(new_n248), .d(new_n249), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n254), .c(new_n188), .d(new_n247), .o1(new_n256));
  aoi112aa1n02x5               g161(.a(new_n255), .b(new_n254), .c(new_n188), .d(new_n247), .o1(new_n257));
  norb02aa1n02x5               g162(.a(new_n256), .b(new_n257), .out0(\s[25] ));
  norp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[26] ), .b(\b[25] ), .out0(new_n260));
  nona22aa1n02x4               g165(.a(new_n256), .b(new_n260), .c(new_n259), .out0(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n259), .clkout(new_n262));
  aobi12aa1n02x5               g167(.a(new_n260), .b(new_n256), .c(new_n262), .out0(new_n263));
  norb02aa1n02x5               g168(.a(new_n261), .b(new_n263), .out0(\s[26] ));
  and002aa1n02x5               g169(.a(new_n260), .b(new_n255), .o(new_n265));
  nano22aa1n02x4               g170(.a(new_n233), .b(new_n245), .c(new_n265), .out0(new_n266));
  nanp02aa1n02x5               g171(.a(new_n188), .b(new_n266), .o1(new_n267));
  oao003aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .c(new_n262), .carry(new_n268));
  aobi12aa1n02x5               g173(.a(new_n268), .b(new_n254), .c(new_n265), .out0(new_n269));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  and002aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o(new_n271));
  norp02aa1n02x5               g176(.a(new_n271), .b(new_n270), .o1(new_n272));
  xnbna2aa1n03x5               g177(.a(new_n272), .b(new_n269), .c(new_n267), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n271), .clkout(new_n274));
  xorc02aa1n02x5               g179(.a(\a[28] ), .b(\b[27] ), .out0(new_n275));
  aobi12aa1n02x5               g180(.a(new_n266), .b(new_n187), .c(new_n183), .out0(new_n276));
  aoai13aa1n02x5               g181(.a(new_n245), .b(new_n235), .c(new_n220), .d(new_n232), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n265), .clkout(new_n278));
  aoai13aa1n02x5               g183(.a(new_n268), .b(new_n278), .c(new_n277), .d(new_n253), .o1(new_n279));
  norp03aa1n02x5               g184(.a(new_n279), .b(new_n276), .c(new_n270), .o1(new_n280));
  nano22aa1n02x4               g185(.a(new_n280), .b(new_n274), .c(new_n275), .out0(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n270), .clkout(new_n282));
  nanp03aa1n02x5               g187(.a(new_n269), .b(new_n267), .c(new_n282), .o1(new_n283));
  aoi012aa1n02x5               g188(.a(new_n275), .b(new_n283), .c(new_n274), .o1(new_n284));
  norp02aa1n02x5               g189(.a(new_n284), .b(new_n281), .o1(\s[28] ));
  and002aa1n02x5               g190(.a(new_n275), .b(new_n272), .o(new_n286));
  oai012aa1n02x5               g191(.a(new_n286), .b(new_n279), .c(new_n276), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[28] ), .b(\a[29] ), .out0(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(new_n287), .c(new_n288), .o1(new_n290));
  aobi12aa1n02x5               g195(.a(new_n286), .b(new_n269), .c(new_n267), .out0(new_n291));
  nano22aa1n02x4               g196(.a(new_n291), .b(new_n288), .c(new_n289), .out0(new_n292));
  norp02aa1n02x5               g197(.a(new_n290), .b(new_n292), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g199(.a(new_n289), .b(new_n275), .c(new_n272), .out0(new_n295));
  oai012aa1n02x5               g200(.a(new_n295), .b(new_n279), .c(new_n276), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n288), .carry(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[29] ), .b(\a[30] ), .out0(new_n298));
  aoi012aa1n02x5               g203(.a(new_n298), .b(new_n296), .c(new_n297), .o1(new_n299));
  aobi12aa1n02x5               g204(.a(new_n295), .b(new_n269), .c(new_n267), .out0(new_n300));
  nano22aa1n02x4               g205(.a(new_n300), .b(new_n297), .c(new_n298), .out0(new_n301));
  norp02aa1n02x5               g206(.a(new_n299), .b(new_n301), .o1(\s[30] ));
  nano23aa1n02x4               g207(.a(new_n298), .b(new_n289), .c(new_n275), .d(new_n272), .out0(new_n303));
  aobi12aa1n02x5               g208(.a(new_n303), .b(new_n269), .c(new_n267), .out0(new_n304));
  oao003aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .c(new_n297), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  nano22aa1n02x4               g211(.a(new_n304), .b(new_n305), .c(new_n306), .out0(new_n307));
  oai012aa1n02x5               g212(.a(new_n303), .b(new_n279), .c(new_n276), .o1(new_n308));
  aoi012aa1n02x5               g213(.a(new_n306), .b(new_n308), .c(new_n305), .o1(new_n309));
  norp02aa1n02x5               g214(.a(new_n309), .b(new_n307), .o1(\s[31] ));
  xnbna2aa1n03x5               g215(.a(new_n109), .b(new_n103), .c(new_n104), .out0(\s[3] ));
  oaoi03aa1n02x5               g216(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g218(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g219(.a(\a[5] ), .b(\b[4] ), .o(new_n315));
  aob012aa1n02x5               g220(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(new_n316));
  xobna2aa1n03x5               g221(.a(new_n117), .b(new_n316), .c(new_n315), .out0(\s[6] ));
  and002aa1n02x5               g222(.a(\b[5] ), .b(\a[6] ), .o(new_n318));
  nona32aa1n02x4               g223(.a(new_n316), .b(new_n123), .c(new_n318), .d(new_n121), .out0(new_n319));
  nona23aa1n02x4               g224(.a(new_n319), .b(new_n115), .c(new_n114), .d(new_n318), .out0(new_n320));
  160nm_ficinv00aa1n08x5       g225(.clk(new_n114), .clkout(new_n321));
  aoi022aa1n02x5               g226(.a(new_n319), .b(new_n122), .c(new_n321), .d(new_n115), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n320), .b(new_n322), .out0(\s[7] ));
  aoi013aa1n02x4               g228(.a(new_n114), .b(new_n319), .c(new_n122), .d(new_n115), .o1(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoai13aa1n02x5               g230(.a(new_n127), .b(new_n126), .c(new_n111), .d(new_n119), .o1(new_n326));
  nanp02aa1n02x5               g231(.a(new_n128), .b(new_n326), .o1(\s[9] ));
endmodule



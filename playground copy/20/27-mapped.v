// Benchmark "adder" written by ABC on Thu Jul 11 12:23:32 2024

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
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n318, new_n320;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  aoi012aa1n02x5               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n109), .b(new_n117), .o1(new_n118));
  orn002aa1n02x5               g023(.a(\a[5] ), .b(\b[4] ), .o(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(new_n112), .clkout(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[8] ), .b(\b[7] ), .c(new_n121), .o1(new_n122));
  aoib12aa1n02x5               g027(.a(new_n122), .b(new_n120), .c(new_n114), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n97), .out0(new_n125));
  160nm_ficinv00aa1n08x5       g030(.clk(new_n125), .clkout(new_n126));
  aoai13aa1n02x5               g031(.a(new_n98), .b(new_n126), .c(new_n118), .d(new_n123), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  oai012aa1n02x5               g039(.a(new_n130), .b(new_n97), .c(new_n129), .o1(new_n135));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n135), .clkout(new_n136));
  aoai13aa1n02x5               g041(.a(new_n134), .b(new_n136), .c(new_n127), .d(new_n131), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n134), .b(new_n136), .c(new_n127), .d(new_n131), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g044(.clk(new_n132), .clkout(new_n140));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n137), .c(new_n140), .out0(\s[12] ));
  nona23aa1n02x4               g049(.a(new_n142), .b(new_n133), .c(new_n132), .d(new_n141), .out0(new_n145));
  nano22aa1n02x4               g050(.a(new_n145), .b(new_n131), .c(new_n125), .out0(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n146), .clkout(new_n147));
  oaoi03aa1n02x5               g052(.a(\a[12] ), .b(\b[11] ), .c(new_n140), .o1(new_n148));
  oabi12aa1n02x5               g053(.a(new_n148), .b(new_n145), .c(new_n135), .out0(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n147), .c(new_n118), .d(new_n123), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  orn002aa1n02x5               g057(.a(\a[13] ), .b(\b[12] ), .o(new_n153));
  xnrc02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .out0(new_n154));
  nanb02aa1n02x5               g059(.a(new_n154), .b(new_n151), .out0(new_n155));
  xnrc02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .out0(new_n156));
  xobna2aa1n03x5               g061(.a(new_n156), .b(new_n155), .c(new_n153), .out0(\s[14] ));
  norp02aa1n02x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  nanb02aa1n02x5               g064(.a(new_n158), .b(new_n159), .out0(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(new_n160), .clkout(new_n161));
  norp02aa1n02x5               g066(.a(new_n156), .b(new_n154), .o1(new_n162));
  oaoi03aa1n02x5               g067(.a(\a[14] ), .b(\b[13] ), .c(new_n153), .o1(new_n163));
  aoai13aa1n02x5               g068(.a(new_n161), .b(new_n163), .c(new_n151), .d(new_n162), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n161), .b(new_n163), .c(new_n151), .d(new_n162), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n158), .clkout(new_n167));
  norp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  xobna2aa1n03x5               g075(.a(new_n170), .b(new_n164), .c(new_n167), .out0(\s[16] ));
  aoi012aa1n02x5               g076(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n172));
  oaib12aa1n02x5               g077(.a(new_n172), .b(new_n114), .c(new_n120), .out0(new_n173));
  aoi012aa1n02x5               g078(.a(new_n173), .b(new_n109), .c(new_n117), .o1(new_n174));
  nona23aa1n02x4               g079(.a(new_n169), .b(new_n159), .c(new_n158), .d(new_n168), .out0(new_n175));
  nona32aa1n02x4               g080(.a(new_n146), .b(new_n175), .c(new_n156), .d(new_n154), .out0(new_n176));
  160nm_ficinv00aa1n08x5       g081(.clk(new_n175), .clkout(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n163), .c(new_n149), .d(new_n162), .o1(new_n178));
  aoi012aa1n02x5               g083(.a(new_n168), .b(new_n158), .c(new_n169), .o1(new_n179));
  oai112aa1n02x5               g084(.a(new_n178), .b(new_n179), .c(new_n174), .d(new_n176), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[18] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\a[17] ), .clkout(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(\b[16] ), .clkout(new_n184));
  oaoi03aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n180), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(new_n182), .out0(\s[18] ));
  aoi012aa1n02x5               g091(.a(new_n176), .b(new_n118), .c(new_n123), .o1(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(new_n163), .clkout(new_n188));
  nano23aa1n02x4               g093(.a(new_n132), .b(new_n141), .c(new_n142), .d(new_n133), .out0(new_n189));
  aoai13aa1n02x5               g094(.a(new_n162), .b(new_n148), .c(new_n189), .d(new_n136), .o1(new_n190));
  aoai13aa1n02x5               g095(.a(new_n179), .b(new_n175), .c(new_n190), .d(new_n188), .o1(new_n191));
  xroi22aa1d04x5               g096(.a(new_n183), .b(\b[16] ), .c(new_n182), .d(\b[17] ), .out0(new_n192));
  oai012aa1n02x5               g097(.a(new_n192), .b(new_n191), .c(new_n187), .o1(new_n193));
  oai022aa1n02x5               g098(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n194));
  oaib12aa1n02x5               g099(.a(new_n194), .b(new_n182), .c(\b[17] ), .out0(new_n195));
  norp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nanb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(new_n198));
  160nm_ficinv00aa1n08x5       g103(.clk(new_n198), .clkout(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n199), .b(new_n193), .c(new_n195), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g106(.clk(new_n196), .clkout(new_n202));
  aoi012aa1n02x5               g107(.a(new_n198), .b(new_n193), .c(new_n195), .o1(new_n203));
  norp02aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  nano22aa1n02x4               g111(.a(new_n203), .b(new_n202), .c(new_n206), .out0(new_n207));
  160nm_ficinv00aa1n08x5       g112(.clk(new_n195), .clkout(new_n208));
  oaoi13aa1n02x5               g113(.a(new_n208), .b(new_n192), .c(new_n191), .d(new_n187), .o1(new_n209));
  oaoi13aa1n02x5               g114(.a(new_n206), .b(new_n202), .c(new_n209), .d(new_n198), .o1(new_n210));
  norp02aa1n02x5               g115(.a(new_n210), .b(new_n207), .o1(\s[20] ));
  nano23aa1n02x4               g116(.a(new_n196), .b(new_n204), .c(new_n205), .d(new_n197), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n192), .b(new_n212), .o1(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  oai012aa1n02x5               g119(.a(new_n214), .b(new_n191), .c(new_n187), .o1(new_n215));
  nona23aa1n02x4               g120(.a(new_n205), .b(new_n197), .c(new_n196), .d(new_n204), .out0(new_n216));
  aoi012aa1n02x5               g121(.a(new_n204), .b(new_n196), .c(new_n205), .o1(new_n217));
  oai012aa1n02x5               g122(.a(new_n217), .b(new_n216), .c(new_n195), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  norp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  xnbna2aa1n03x5               g127(.a(new_n222), .b(new_n215), .c(new_n219), .out0(\s[21] ));
  160nm_ficinv00aa1n08x5       g128(.clk(new_n220), .clkout(new_n224));
  aobi12aa1n02x5               g129(.a(new_n222), .b(new_n215), .c(new_n219), .out0(new_n225));
  xnrc02aa1n02x5               g130(.a(\b[21] ), .b(\a[22] ), .out0(new_n226));
  nano22aa1n02x4               g131(.a(new_n225), .b(new_n224), .c(new_n226), .out0(new_n227));
  aoai13aa1n02x5               g132(.a(new_n222), .b(new_n218), .c(new_n180), .d(new_n214), .o1(new_n228));
  aoi012aa1n02x5               g133(.a(new_n226), .b(new_n228), .c(new_n224), .o1(new_n229));
  norp02aa1n02x5               g134(.a(new_n229), .b(new_n227), .o1(\s[22] ));
  nano22aa1n02x4               g135(.a(new_n226), .b(new_n224), .c(new_n221), .out0(new_n231));
  and003aa1n02x5               g136(.a(new_n192), .b(new_n231), .c(new_n212), .o(new_n232));
  oai012aa1n02x5               g137(.a(new_n232), .b(new_n191), .c(new_n187), .o1(new_n233));
  oao003aa1n02x5               g138(.a(\a[22] ), .b(\b[21] ), .c(new_n224), .carry(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n234), .clkout(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n218), .c(new_n231), .o1(new_n236));
  xnrc02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .out0(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n237), .clkout(new_n238));
  xnbna2aa1n03x5               g143(.a(new_n238), .b(new_n233), .c(new_n236), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  aoi012aa1n02x5               g146(.a(new_n237), .b(new_n233), .c(new_n236), .o1(new_n242));
  xnrc02aa1n02x5               g147(.a(\b[23] ), .b(\a[24] ), .out0(new_n243));
  nano22aa1n02x4               g148(.a(new_n242), .b(new_n241), .c(new_n243), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n236), .clkout(new_n245));
  oaoi13aa1n02x5               g150(.a(new_n245), .b(new_n232), .c(new_n191), .d(new_n187), .o1(new_n246));
  oaoi13aa1n02x5               g151(.a(new_n243), .b(new_n241), .c(new_n246), .d(new_n237), .o1(new_n247));
  norp02aa1n02x5               g152(.a(new_n247), .b(new_n244), .o1(\s[24] ));
  norp02aa1n02x5               g153(.a(new_n243), .b(new_n237), .o1(new_n249));
  nano22aa1n02x4               g154(.a(new_n213), .b(new_n231), .c(new_n249), .out0(new_n250));
  oai012aa1n02x5               g155(.a(new_n250), .b(new_n191), .c(new_n187), .o1(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n217), .clkout(new_n252));
  aoai13aa1n02x5               g157(.a(new_n231), .b(new_n252), .c(new_n212), .d(new_n208), .o1(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n249), .clkout(new_n254));
  oao003aa1n02x5               g159(.a(\a[24] ), .b(\b[23] ), .c(new_n241), .carry(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n254), .c(new_n253), .d(new_n234), .o1(new_n256));
  xnrc02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .out0(new_n257));
  aoib12aa1n02x5               g162(.a(new_n257), .b(new_n251), .c(new_n256), .out0(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n257), .clkout(new_n259));
  aoi112aa1n02x5               g164(.a(new_n259), .b(new_n256), .c(new_n180), .d(new_n250), .o1(new_n260));
  norp02aa1n02x5               g165(.a(new_n258), .b(new_n260), .o1(\s[25] ));
  norp02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .o1(new_n262));
  160nm_ficinv00aa1n08x5       g167(.clk(new_n262), .clkout(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[25] ), .b(\a[26] ), .out0(new_n264));
  nano22aa1n02x4               g169(.a(new_n258), .b(new_n263), .c(new_n264), .out0(new_n265));
  oaoi13aa1n02x5               g170(.a(new_n256), .b(new_n250), .c(new_n191), .d(new_n187), .o1(new_n266));
  oaoi13aa1n02x5               g171(.a(new_n264), .b(new_n263), .c(new_n266), .d(new_n257), .o1(new_n267));
  norp02aa1n02x5               g172(.a(new_n267), .b(new_n265), .o1(\s[26] ));
  norp02aa1n02x5               g173(.a(new_n264), .b(new_n257), .o1(new_n269));
  nano32aa1n02x4               g174(.a(new_n213), .b(new_n269), .c(new_n231), .d(new_n249), .out0(new_n270));
  oai012aa1n02x5               g175(.a(new_n270), .b(new_n191), .c(new_n187), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .c(new_n263), .carry(new_n272));
  aobi12aa1n02x5               g177(.a(new_n272), .b(new_n256), .c(new_n269), .out0(new_n273));
  xorc02aa1n02x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n273), .c(new_n271), .out0(\s[27] ));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n276), .clkout(new_n277));
  aobi12aa1n02x5               g182(.a(new_n274), .b(new_n273), .c(new_n271), .out0(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .out0(new_n279));
  nano22aa1n02x4               g184(.a(new_n278), .b(new_n277), .c(new_n279), .out0(new_n280));
  aoai13aa1n02x5               g185(.a(new_n249), .b(new_n235), .c(new_n218), .d(new_n231), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n269), .clkout(new_n282));
  aoai13aa1n02x5               g187(.a(new_n272), .b(new_n282), .c(new_n281), .d(new_n255), .o1(new_n283));
  aoai13aa1n02x5               g188(.a(new_n274), .b(new_n283), .c(new_n180), .d(new_n270), .o1(new_n284));
  aoi012aa1n02x5               g189(.a(new_n279), .b(new_n284), .c(new_n277), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n285), .b(new_n280), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n274), .b(new_n279), .out0(new_n287));
  aoai13aa1n02x5               g192(.a(new_n287), .b(new_n283), .c(new_n180), .d(new_n270), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  aobi12aa1n02x5               g196(.a(new_n287), .b(new_n273), .c(new_n271), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n292), .b(new_n289), .c(new_n290), .out0(new_n293));
  norp02aa1n02x5               g198(.a(new_n291), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n274), .b(new_n290), .c(new_n279), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n283), .c(new_n180), .d(new_n270), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  aoi012aa1n02x5               g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n02x5               g205(.a(new_n296), .b(new_n273), .c(new_n271), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  norp02aa1n02x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  norb02aa1n02x5               g209(.a(new_n296), .b(new_n299), .out0(new_n305));
  aobi12aa1n02x5               g210(.a(new_n305), .b(new_n273), .c(new_n271), .out0(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n307));
  nano22aa1n02x4               g212(.a(new_n306), .b(new_n304), .c(new_n307), .out0(new_n308));
  aoai13aa1n02x5               g213(.a(new_n305), .b(new_n283), .c(new_n180), .d(new_n270), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n304), .b(new_n309), .c(new_n307), .o1(new_n310));
  norp02aa1n02x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaib12aa1n02x5               g220(.a(new_n119), .b(new_n116), .c(new_n109), .out0(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoib12aa1n02x5               g222(.a(new_n120), .b(new_n316), .c(new_n115), .out0(new_n318));
  xnbna2aa1n03x5               g223(.a(new_n318), .b(new_n121), .c(new_n113), .out0(\s[7] ));
  oaoi03aa1n02x5               g224(.a(\a[7] ), .b(\b[6] ), .c(new_n318), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g226(.a(new_n125), .b(new_n118), .c(new_n123), .out0(\s[9] ));
endmodule



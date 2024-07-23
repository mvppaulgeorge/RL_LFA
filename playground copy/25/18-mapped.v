// Benchmark "adder" written by ABC on Thu Jul 11 12:44:25 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n315, new_n317, new_n318, new_n319, new_n321;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  oai012aa1n02x5               g011(.a(new_n103), .b(new_n104), .c(new_n102), .o1(new_n107));
  oai012aa1n02x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nona23aa1n02x4               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  norp02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nanb02aa1n02x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  norp03aa1n02x5               g022(.a(new_n113), .b(new_n116), .c(new_n117), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(new_n108), .b(new_n118), .o1(new_n119));
  norp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  aoi012aa1n02x5               g025(.a(new_n114), .b(new_n120), .c(new_n115), .o1(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(new_n111), .clkout(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[8] ), .b(\b[7] ), .c(new_n122), .o1(new_n123));
  oabi12aa1n02x5               g028(.a(new_n123), .b(new_n113), .c(new_n121), .out0(new_n124));
  160nm_ficinv00aa1n08x5       g029(.clk(new_n124), .clkout(new_n125));
  nanp02aa1n02x5               g030(.a(new_n119), .b(new_n125), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n97), .b(new_n126), .c(new_n127), .o1(new_n128));
  xnrb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  norp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  nano23aa1n02x4               g039(.a(new_n97), .b(new_n133), .c(new_n134), .d(new_n127), .out0(new_n135));
  160nm_fiao0012aa1n02p5x5     g040(.a(new_n133), .b(new_n97), .c(new_n134), .o(new_n136));
  aoai13aa1n02x5               g041(.a(new_n132), .b(new_n136), .c(new_n126), .d(new_n135), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n132), .b(new_n136), .c(new_n126), .d(new_n135), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  orn002aa1n02x5               g044(.a(\a[11] ), .b(\b[10] ), .o(new_n140));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n137), .c(new_n140), .out0(\s[12] ));
  nano23aa1n02x4               g049(.a(new_n130), .b(new_n141), .c(new_n142), .d(new_n131), .out0(new_n145));
  nanp02aa1n02x5               g050(.a(new_n145), .b(new_n135), .o1(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n146), .clkout(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n124), .c(new_n108), .d(new_n118), .o1(new_n148));
  aoi112aa1n02x5               g053(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n149));
  aoi112aa1n02x5               g054(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n150));
  oai112aa1n02x5               g055(.a(new_n143), .b(new_n132), .c(new_n150), .d(new_n133), .o1(new_n151));
  nona22aa1n02x4               g056(.a(new_n151), .b(new_n149), .c(new_n141), .out0(new_n152));
  160nm_ficinv00aa1n08x5       g057(.clk(new_n152), .clkout(new_n153));
  norp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n148), .c(new_n153), .out0(\s[13] ));
  nanp02aa1n02x5               g062(.a(new_n148), .b(new_n153), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n154), .b(new_n158), .c(new_n155), .o1(new_n159));
  norp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  xnrc02aa1n02x5               g067(.a(new_n159), .b(new_n162), .out0(\s[14] ));
  nona23aa1n02x4               g068(.a(new_n161), .b(new_n155), .c(new_n154), .d(new_n160), .out0(new_n164));
  aoi012aa1n02x5               g069(.a(new_n160), .b(new_n154), .c(new_n161), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n164), .c(new_n148), .d(new_n153), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  160nm_ficinv00aa1n08x5       g077(.clk(new_n172), .clkout(new_n173));
  aoi112aa1n02x5               g078(.a(new_n173), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n173), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(\s[16] ));
  nano23aa1n02x4               g081(.a(new_n168), .b(new_n170), .c(new_n171), .d(new_n169), .out0(new_n177));
  nano32aa1n02x4               g082(.a(new_n146), .b(new_n177), .c(new_n156), .d(new_n162), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n124), .c(new_n108), .d(new_n118), .o1(new_n179));
  nanb02aa1n02x5               g084(.a(new_n168), .b(new_n169), .out0(new_n180));
  norp03aa1n02x5               g085(.a(new_n164), .b(new_n172), .c(new_n180), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n170), .clkout(new_n183));
  oai013aa1n02x4               g088(.a(new_n183), .b(new_n165), .c(new_n180), .d(new_n172), .o1(new_n184));
  aoi112aa1n02x5               g089(.a(new_n184), .b(new_n182), .c(new_n152), .d(new_n181), .o1(new_n185));
  xorc02aa1n02x5               g090(.a(\a[17] ), .b(\b[16] ), .out0(new_n186));
  xnbna2aa1n03x5               g091(.a(new_n186), .b(new_n185), .c(new_n179), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[18] ), .clkout(new_n188));
  nanp02aa1n02x5               g093(.a(new_n185), .b(new_n179), .o1(new_n189));
  norp02aa1n02x5               g094(.a(\b[16] ), .b(\a[17] ), .o1(new_n190));
  aoi012aa1n02x5               g095(.a(new_n190), .b(new_n189), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g097(.clk(\a[17] ), .clkout(new_n193));
  xroi22aa1d04x5               g098(.a(new_n193), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(new_n194), .clkout(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(\b[17] ), .clkout(new_n196));
  oao003aa1n02x5               g101(.a(new_n188), .b(new_n196), .c(new_n190), .carry(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(new_n197), .clkout(new_n198));
  aoai13aa1n02x5               g103(.a(new_n198), .b(new_n195), .c(new_n185), .d(new_n179), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  norp02aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  aoi112aa1n02x5               g111(.a(new_n202), .b(new_n206), .c(new_n199), .d(new_n203), .o1(new_n207));
  aoai13aa1n02x5               g112(.a(new_n206), .b(new_n202), .c(new_n199), .d(new_n203), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n208), .b(new_n207), .out0(\s[20] ));
  nano23aa1n02x4               g114(.a(new_n202), .b(new_n204), .c(new_n205), .d(new_n203), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n194), .b(new_n210), .o1(new_n211));
  aoi112aa1n02x5               g116(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n212));
  aoi112aa1n02x5               g117(.a(new_n212), .b(new_n204), .c(new_n210), .d(new_n197), .o1(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n211), .c(new_n185), .d(new_n179), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xnrc02aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .out0(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  xnrc02aa1n02x5               g123(.a(\b[21] ), .b(\a[22] ), .out0(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n219), .clkout(new_n220));
  aoi112aa1n02x5               g125(.a(new_n216), .b(new_n220), .c(new_n214), .d(new_n218), .o1(new_n221));
  aoai13aa1n02x5               g126(.a(new_n220), .b(new_n216), .c(new_n214), .d(new_n218), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(\s[22] ));
  norp02aa1n02x5               g128(.a(new_n219), .b(new_n217), .o1(new_n224));
  nanp03aa1n02x5               g129(.a(new_n194), .b(new_n224), .c(new_n210), .o1(new_n225));
  norp02aa1n02x5               g130(.a(\b[17] ), .b(\a[18] ), .o1(new_n226));
  aoi112aa1n02x5               g131(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n203), .b(new_n202), .out0(new_n228));
  oai112aa1n02x5               g133(.a(new_n228), .b(new_n206), .c(new_n227), .d(new_n226), .o1(new_n229));
  nona22aa1n02x4               g134(.a(new_n229), .b(new_n212), .c(new_n204), .out0(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(new_n216), .clkout(new_n231));
  oaoi03aa1n02x5               g136(.a(\a[22] ), .b(\b[21] ), .c(new_n231), .o1(new_n232));
  aoi012aa1n02x5               g137(.a(new_n232), .b(new_n230), .c(new_n224), .o1(new_n233));
  aoai13aa1n02x5               g138(.a(new_n233), .b(new_n225), .c(new_n185), .d(new_n179), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  norp02aa1n02x5               g142(.a(\b[23] ), .b(\a[24] ), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(\b[23] ), .b(\a[24] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  aoi112aa1n02x5               g145(.a(new_n236), .b(new_n240), .c(new_n234), .d(new_n237), .o1(new_n241));
  aoai13aa1n02x5               g146(.a(new_n240), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(\s[24] ));
  nona23aa1n02x4               g148(.a(new_n239), .b(new_n237), .c(new_n236), .d(new_n238), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  nanb03aa1n02x5               g150(.a(new_n211), .b(new_n245), .c(new_n224), .out0(new_n246));
  nona32aa1n02x4               g151(.a(new_n230), .b(new_n244), .c(new_n219), .d(new_n217), .out0(new_n247));
  aoi112aa1n02x5               g152(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n248), .b(new_n238), .c(new_n245), .d(new_n232), .o1(new_n249));
  nanp02aa1n02x5               g154(.a(new_n247), .b(new_n249), .o1(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n250), .clkout(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n246), .c(new_n185), .d(new_n179), .o1(new_n252));
  xorb03aa1n02x5               g157(.a(new_n252), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  xorc02aa1n02x5               g160(.a(\a[26] ), .b(\b[25] ), .out0(new_n256));
  aoi112aa1n02x5               g161(.a(new_n254), .b(new_n256), .c(new_n252), .d(new_n255), .o1(new_n257));
  aoai13aa1n02x5               g162(.a(new_n256), .b(new_n254), .c(new_n252), .d(new_n255), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n258), .b(new_n257), .out0(\s[26] ));
  nanp02aa1n02x5               g164(.a(new_n152), .b(new_n181), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n260), .b(new_n184), .c(new_n182), .out0(new_n261));
  and002aa1n02x5               g166(.a(new_n256), .b(new_n255), .o(new_n262));
  nano22aa1n02x4               g167(.a(new_n225), .b(new_n262), .c(new_n245), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n261), .c(new_n126), .d(new_n178), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .o1(new_n265));
  oai022aa1n02x5               g170(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n266));
  aoi022aa1n02x5               g171(.a(new_n250), .b(new_n262), .c(new_n265), .d(new_n266), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  xnbna2aa1n03x5               g173(.a(new_n268), .b(new_n264), .c(new_n267), .out0(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n270), .clkout(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n268), .clkout(new_n272));
  aoi012aa1n02x5               g177(.a(new_n272), .b(new_n264), .c(new_n267), .o1(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[27] ), .b(\a[28] ), .out0(new_n274));
  nano22aa1n02x4               g179(.a(new_n273), .b(new_n271), .c(new_n274), .out0(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n262), .clkout(new_n276));
  nanp02aa1n02x5               g181(.a(new_n266), .b(new_n265), .o1(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n276), .c(new_n247), .d(new_n249), .o1(new_n278));
  aoai13aa1n02x5               g183(.a(new_n268), .b(new_n278), .c(new_n189), .d(new_n263), .o1(new_n279));
  aoi012aa1n02x5               g184(.a(new_n274), .b(new_n279), .c(new_n271), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n280), .b(new_n275), .o1(\s[28] ));
  norb02aa1n02x5               g186(.a(new_n268), .b(new_n274), .out0(new_n282));
  aoai13aa1n02x5               g187(.a(new_n282), .b(new_n278), .c(new_n189), .d(new_n263), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  aoi012aa1n02x5               g190(.a(new_n285), .b(new_n283), .c(new_n284), .o1(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n282), .clkout(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(new_n264), .c(new_n267), .o1(new_n288));
  nano22aa1n02x4               g193(.a(new_n288), .b(new_n284), .c(new_n285), .out0(new_n289));
  norp02aa1n02x5               g194(.a(new_n286), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g196(.a(new_n268), .b(new_n285), .c(new_n274), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n278), .c(new_n189), .d(new_n263), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  160nm_ficinv00aa1n08x5       g201(.clk(new_n292), .clkout(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n264), .c(new_n267), .o1(new_n298));
  nano22aa1n02x4               g203(.a(new_n298), .b(new_n294), .c(new_n295), .out0(new_n299));
  norp02aa1n02x5               g204(.a(new_n296), .b(new_n299), .o1(\s[30] ));
  norb02aa1n02x5               g205(.a(new_n292), .b(new_n295), .out0(new_n301));
  160nm_ficinv00aa1n08x5       g206(.clk(new_n301), .clkout(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n264), .c(new_n267), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  aoai13aa1n02x5               g211(.a(new_n301), .b(new_n278), .c(new_n189), .d(new_n263), .o1(new_n307));
  aoi012aa1n02x5               g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g214(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g215(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g218(.a(new_n117), .b(new_n108), .out0(new_n314));
  oai012aa1n02x5               g219(.a(new_n314), .b(\b[4] ), .c(\a[5] ), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g221(.a(new_n112), .b(new_n111), .out0(new_n317));
  aoai13aa1n02x5               g222(.a(new_n317), .b(new_n114), .c(new_n315), .d(new_n115), .o1(new_n318));
  aoi112aa1n02x5               g223(.a(new_n317), .b(new_n114), .c(new_n315), .d(new_n115), .o1(new_n319));
  norb02aa1n02x5               g224(.a(new_n318), .b(new_n319), .out0(\s[7] ));
  norb02aa1n02x5               g225(.a(new_n110), .b(new_n109), .out0(new_n321));
  xnbna2aa1n03x5               g226(.a(new_n321), .b(new_n318), .c(new_n122), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule



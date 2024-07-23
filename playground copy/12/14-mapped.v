// Benchmark "adder" written by ABC on Thu Jul 11 11:45:57 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n315, new_n318, new_n320,
    new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  and002aa1n02x5               g002(.a(\b[3] ), .b(\a[4] ), .o(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\a[3] ), .clkout(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\b[2] ), .clkout(new_n100));
  nanp02aa1n02x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(new_n101), .b(new_n102), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  oai012aa1n02x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  oa0022aa1n02x5               g012(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n103), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp02aa1n02x5               g021(.a(new_n116), .b(new_n115), .o1(new_n117));
  nona23aa1n02x4               g022(.a(new_n109), .b(new_n117), .c(new_n114), .d(new_n98), .out0(new_n118));
  nano23aa1n02x4               g023(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n119));
  orn002aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .o(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n122));
  aobi12aa1n02x5               g027(.a(new_n122), .b(new_n119), .c(new_n121), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(new_n118), .b(new_n123), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  aoi012aa1n02x5               g030(.a(new_n97), .b(new_n124), .c(new_n125), .o1(new_n126));
  xnrb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanb02aa1n02x5               g034(.a(new_n128), .b(new_n129), .out0(new_n130));
  160nm_ficinv00aa1n08x5       g035(.clk(new_n130), .clkout(new_n131));
  norp02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nano23aa1n02x4               g038(.a(new_n132), .b(new_n97), .c(new_n125), .d(new_n133), .out0(new_n134));
  aoi012aa1n02x5               g039(.a(new_n132), .b(new_n97), .c(new_n133), .o1(new_n135));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n135), .clkout(new_n136));
  aoai13aa1n02x5               g041(.a(new_n131), .b(new_n136), .c(new_n124), .d(new_n134), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n131), .b(new_n136), .c(new_n124), .d(new_n134), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  oai012aa1n02x5               g044(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nano23aa1n02x4               g048(.a(new_n128), .b(new_n142), .c(new_n143), .d(new_n129), .out0(new_n144));
  nanp02aa1n02x5               g049(.a(new_n144), .b(new_n134), .o1(new_n145));
  nona23aa1n02x4               g050(.a(new_n143), .b(new_n129), .c(new_n128), .d(new_n142), .out0(new_n146));
  oai012aa1n02x5               g051(.a(new_n143), .b(new_n142), .c(new_n128), .o1(new_n147));
  oai012aa1n02x5               g052(.a(new_n147), .b(new_n146), .c(new_n135), .o1(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n145), .c(new_n118), .d(new_n123), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .o1(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oaoi13aa1n02x5               g060(.a(new_n98), .b(new_n108), .c(new_n107), .d(new_n103), .o1(new_n156));
  norp03aa1n02x5               g061(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n157));
  oaib12aa1n02x5               g062(.a(new_n122), .b(new_n114), .c(new_n121), .out0(new_n158));
  norb02aa1n02x5               g063(.a(new_n134), .b(new_n146), .out0(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n158), .c(new_n156), .d(new_n157), .o1(new_n160));
  norp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nona23aa1n02x4               g067(.a(new_n162), .b(new_n153), .c(new_n152), .d(new_n161), .out0(new_n163));
  aoi012aa1n02x5               g068(.a(new_n161), .b(new_n152), .c(new_n162), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n160), .d(new_n149), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  xnrc02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .out0(new_n168));
  160nm_ficinv00aa1n08x5       g073(.clk(new_n168), .clkout(new_n169));
  xnrc02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .out0(new_n170));
  160nm_ficinv00aa1n08x5       g075(.clk(new_n170), .clkout(new_n171));
  aoi112aa1n02x5               g076(.a(new_n171), .b(new_n167), .c(new_n165), .d(new_n169), .o1(new_n172));
  160nm_ficinv00aa1n08x5       g077(.clk(new_n167), .clkout(new_n173));
  nano23aa1n02x4               g078(.a(new_n152), .b(new_n161), .c(new_n162), .d(new_n153), .out0(new_n174));
  160nm_ficinv00aa1n08x5       g079(.clk(new_n164), .clkout(new_n175));
  aoai13aa1n02x5               g080(.a(new_n169), .b(new_n175), .c(new_n150), .d(new_n174), .o1(new_n176));
  aoi012aa1n02x5               g081(.a(new_n170), .b(new_n176), .c(new_n173), .o1(new_n177));
  norp02aa1n02x5               g082(.a(new_n177), .b(new_n172), .o1(\s[16] ));
  norp03aa1n02x5               g083(.a(new_n163), .b(new_n170), .c(new_n168), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(new_n159), .b(new_n179), .o1(new_n180));
  oao003aa1n02x5               g085(.a(\a[16] ), .b(\b[15] ), .c(new_n173), .carry(new_n181));
  oai013aa1n02x4               g086(.a(new_n181), .b(new_n170), .c(new_n168), .d(new_n164), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n182), .b(new_n148), .c(new_n179), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n180), .c(new_n118), .d(new_n123), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[18] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[17] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\b[16] ), .clkout(new_n188));
  oaoi03aa1n02x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  nano32aa1n02x4               g095(.a(new_n145), .b(new_n171), .c(new_n169), .d(new_n174), .out0(new_n191));
  aoai13aa1n02x5               g096(.a(new_n191), .b(new_n158), .c(new_n157), .d(new_n156), .o1(new_n192));
  xroi22aa1d04x5               g097(.a(new_n187), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n193), .clkout(new_n194));
  norp02aa1n02x5               g099(.a(\b[16] ), .b(\a[17] ), .o1(new_n195));
  norp02aa1n02x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  aoi012aa1n02x5               g102(.a(new_n196), .b(new_n195), .c(new_n197), .o1(new_n198));
  aoai13aa1n02x5               g103(.a(new_n198), .b(new_n194), .c(new_n192), .d(new_n183), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  norp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  aoi112aa1n02x5               g112(.a(new_n202), .b(new_n207), .c(new_n199), .d(new_n204), .o1(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n202), .clkout(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n198), .clkout(new_n210));
  aoai13aa1n02x5               g115(.a(new_n204), .b(new_n210), .c(new_n184), .d(new_n193), .o1(new_n211));
  aobi12aa1n02x5               g116(.a(new_n207), .b(new_n211), .c(new_n209), .out0(new_n212));
  norp02aa1n02x5               g117(.a(new_n212), .b(new_n208), .o1(\s[20] ));
  nona23aa1n02x4               g118(.a(new_n206), .b(new_n203), .c(new_n202), .d(new_n205), .out0(new_n214));
  norb02aa1n02x5               g119(.a(new_n193), .b(new_n214), .out0(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  oai012aa1n02x5               g121(.a(new_n206), .b(new_n205), .c(new_n202), .o1(new_n217));
  oai012aa1n02x5               g122(.a(new_n217), .b(new_n214), .c(new_n198), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n216), .c(new_n192), .d(new_n183), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xnrc02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .out0(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(new_n223), .clkout(new_n224));
  xnrc02aa1n02x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n225), .clkout(new_n226));
  aoi112aa1n02x5               g131(.a(new_n222), .b(new_n226), .c(new_n220), .d(new_n224), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n224), .b(new_n218), .c(new_n184), .d(new_n215), .o1(new_n228));
  oaoi13aa1n02x5               g133(.a(new_n225), .b(new_n228), .c(\a[21] ), .d(\b[20] ), .o1(new_n229));
  norp02aa1n02x5               g134(.a(new_n229), .b(new_n227), .o1(\s[22] ));
  160nm_ficinv00aa1n08x5       g135(.clk(new_n214), .clkout(new_n231));
  norp02aa1n02x5               g136(.a(new_n225), .b(new_n223), .o1(new_n232));
  nanp03aa1n02x5               g137(.a(new_n193), .b(new_n231), .c(new_n232), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n222), .clkout(new_n234));
  oaoi03aa1n02x5               g139(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n218), .c(new_n232), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n233), .c(new_n192), .d(new_n183), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  nanp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  norb02aa1n02x5               g145(.a(new_n240), .b(new_n239), .out0(new_n241));
  norp02aa1n02x5               g146(.a(\b[23] ), .b(\a[24] ), .o1(new_n242));
  nanp02aa1n02x5               g147(.a(\b[23] ), .b(\a[24] ), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(new_n244));
  aoi112aa1n02x5               g149(.a(new_n239), .b(new_n244), .c(new_n237), .d(new_n241), .o1(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n239), .clkout(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n233), .clkout(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n236), .clkout(new_n248));
  aoai13aa1n02x5               g153(.a(new_n241), .b(new_n248), .c(new_n184), .d(new_n247), .o1(new_n249));
  aobi12aa1n02x5               g154(.a(new_n244), .b(new_n249), .c(new_n246), .out0(new_n250));
  norp02aa1n02x5               g155(.a(new_n250), .b(new_n245), .o1(\s[24] ));
  nona23aa1n02x4               g156(.a(new_n243), .b(new_n240), .c(new_n239), .d(new_n242), .out0(new_n252));
  nano32aa1n02x4               g157(.a(new_n252), .b(new_n193), .c(new_n232), .d(new_n231), .out0(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n253), .clkout(new_n254));
  nona32aa1n02x4               g159(.a(new_n218), .b(new_n252), .c(new_n225), .d(new_n223), .out0(new_n255));
  oaoi03aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .c(new_n246), .o1(new_n256));
  aoi013aa1n02x4               g161(.a(new_n256), .b(new_n235), .c(new_n241), .d(new_n244), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(new_n255), .b(new_n257), .o1(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n258), .clkout(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n254), .c(new_n192), .d(new_n183), .o1(new_n260));
  xorb03aa1n02x5               g165(.a(new_n260), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .o1(new_n262));
  xorc02aa1n02x5               g167(.a(\a[25] ), .b(\b[24] ), .out0(new_n263));
  xorc02aa1n02x5               g168(.a(\a[26] ), .b(\b[25] ), .out0(new_n264));
  aoi112aa1n02x5               g169(.a(new_n262), .b(new_n264), .c(new_n260), .d(new_n263), .o1(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n262), .clkout(new_n266));
  aoai13aa1n02x5               g171(.a(new_n263), .b(new_n258), .c(new_n184), .d(new_n253), .o1(new_n267));
  aobi12aa1n02x5               g172(.a(new_n264), .b(new_n267), .c(new_n266), .out0(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n265), .o1(\s[26] ));
  nanp02aa1n02x5               g174(.a(new_n264), .b(new_n263), .o1(new_n270));
  norp03aa1n02x5               g175(.a(new_n233), .b(new_n252), .c(new_n270), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .c(new_n266), .carry(new_n272));
  aoai13aa1n02x5               g177(.a(new_n272), .b(new_n270), .c(new_n255), .d(new_n257), .o1(new_n273));
  aoi012aa1n02x5               g178(.a(new_n273), .b(new_n184), .c(new_n271), .o1(new_n274));
  xnrb03aa1n02x5               g179(.a(new_n274), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nanp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(\a[28] ), .clkout(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(\b[27] ), .clkout(new_n278));
  nanp02aa1n02x5               g183(.a(new_n278), .b(new_n277), .o1(new_n279));
  nanp02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .o1(new_n280));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  aoi112aa1n02x5               g186(.a(new_n273), .b(new_n281), .c(new_n184), .d(new_n271), .o1(new_n282));
  nano32aa1n02x4               g187(.a(new_n282), .b(new_n280), .c(new_n279), .d(new_n276), .out0(new_n283));
  nanp02aa1n02x5               g188(.a(new_n184), .b(new_n271), .o1(new_n284));
  nona22aa1n02x4               g189(.a(new_n284), .b(new_n273), .c(new_n281), .out0(new_n285));
  aoi022aa1n02x5               g190(.a(new_n285), .b(new_n276), .c(new_n279), .d(new_n280), .o1(new_n286));
  norp02aa1n02x5               g191(.a(new_n286), .b(new_n283), .o1(\s[28] ));
  nano32aa1n02x4               g192(.a(new_n281), .b(new_n280), .c(new_n276), .d(new_n279), .out0(new_n288));
  aoai13aa1n02x5               g193(.a(new_n288), .b(new_n273), .c(new_n184), .d(new_n271), .o1(new_n289));
  oaoi03aa1n02x5               g194(.a(new_n277), .b(new_n278), .c(new_n281), .o1(new_n290));
  xorc02aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .out0(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(new_n291), .clkout(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n289), .c(new_n290), .o1(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n290), .clkout(new_n294));
  nona22aa1n02x4               g199(.a(new_n289), .b(new_n294), .c(new_n291), .out0(new_n295));
  norb02aa1n02x5               g200(.a(new_n295), .b(new_n293), .out0(\s[29] ));
  xorb03aa1n02x5               g201(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb02aa1n02x5               g202(.a(new_n288), .b(new_n292), .out0(new_n298));
  aoai13aa1n02x5               g203(.a(new_n298), .b(new_n273), .c(new_n184), .d(new_n271), .o1(new_n299));
  oaoi03aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .o1(new_n300));
  160nm_ficinv00aa1n08x5       g205(.clk(new_n300), .clkout(new_n301));
  xorc02aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .out0(new_n302));
  aobi12aa1n02x5               g207(.a(new_n302), .b(new_n299), .c(new_n301), .out0(new_n303));
  nona22aa1n02x4               g208(.a(new_n299), .b(new_n300), .c(new_n302), .out0(new_n304));
  norb02aa1n02x5               g209(.a(new_n304), .b(new_n303), .out0(\s[30] ));
  and003aa1n02x5               g210(.a(new_n288), .b(new_n302), .c(new_n291), .o(new_n306));
  aoai13aa1n02x5               g211(.a(new_n306), .b(new_n273), .c(new_n184), .d(new_n271), .o1(new_n307));
  oaoi03aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .o1(new_n308));
  xorc02aa1n02x5               g213(.a(\a[31] ), .b(\b[30] ), .out0(new_n309));
  nona22aa1n02x4               g214(.a(new_n307), .b(new_n308), .c(new_n309), .out0(new_n310));
  160nm_ficinv00aa1n08x5       g215(.clk(new_n308), .clkout(new_n311));
  aobi12aa1n02x5               g216(.a(new_n309), .b(new_n307), .c(new_n311), .out0(new_n312));
  norb02aa1n02x5               g217(.a(new_n310), .b(new_n312), .out0(\s[31] ));
  xnbna2aa1n03x5               g218(.a(new_n107), .b(new_n101), .c(new_n102), .out0(\s[3] ));
  oaoi03aa1n02x5               g219(.a(\a[3] ), .b(\b[2] ), .c(new_n107), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g221(.a(new_n156), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aob012aa1n02x5               g222(.a(new_n156), .b(\b[4] ), .c(\a[5] ), .out0(new_n318));
  xobna2aa1n03x5               g223(.a(new_n115), .b(new_n318), .c(new_n120), .out0(\s[6] ));
  aoi012aa1n02x5               g224(.a(new_n121), .b(new_n156), .c(new_n117), .o1(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g226(.a(\a[7] ), .b(\b[6] ), .c(new_n320), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule



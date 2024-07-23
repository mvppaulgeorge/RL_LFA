// Benchmark "adder" written by ABC on Thu Jul 11 11:14:51 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n306, new_n309, new_n310, new_n312, new_n313,
    new_n315;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  and002aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o(new_n100));
  and002aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\a[3] ), .clkout(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\b[2] ), .clkout(new_n103));
  nanp02aa1n02x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  aoi012aa1n02x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  oa0022aa1n02x5               g015(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n111));
  oaoi13aa1n02x5               g016(.a(new_n101), .b(new_n111), .c(new_n110), .d(new_n106), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .out0(new_n118));
  xnrc02aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .out0(new_n119));
  norp03aa1n02x5               g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(new_n113), .clkout(new_n121));
  nanp02aa1n02x5               g026(.a(new_n115), .b(new_n114), .o1(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(\a[6] ), .clkout(new_n123));
  oai022aa1n02x5               g028(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  oaib12aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(\b[5] ), .out0(new_n125));
  oai112aa1n02x5               g030(.a(new_n121), .b(new_n122), .c(new_n117), .d(new_n125), .o1(new_n126));
  aoi012aa1n02x5               g031(.a(new_n126), .b(new_n112), .c(new_n120), .o1(new_n127));
  norp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  norp02aa1n02x5               g033(.a(new_n100), .b(new_n128), .o1(new_n129));
  aoai13aa1n02x5               g034(.a(new_n99), .b(new_n100), .c(new_n127), .d(new_n129), .o1(new_n130));
  aoi112aa1n02x5               g035(.a(new_n99), .b(new_n100), .c(new_n127), .d(new_n129), .o1(new_n131));
  nanb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(\s[10] ));
  norp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n130), .c(new_n98), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n133), .clkout(new_n137));
  160nm_ficinv00aa1n08x5       g042(.clk(new_n98), .clkout(new_n138));
  nona22aa1n02x4               g043(.a(new_n130), .b(new_n135), .c(new_n138), .out0(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(new_n142));
  xobna2aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n137), .out0(\s[12] ));
  nano23aa1n02x4               g048(.a(new_n142), .b(new_n135), .c(new_n129), .d(new_n99), .out0(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n145));
  aoi112aa1n02x5               g050(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n146));
  oai022aa1n02x5               g051(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n147));
  nano23aa1n02x4               g052(.a(new_n142), .b(new_n135), .c(new_n147), .d(new_n98), .out0(new_n148));
  norp03aa1n02x5               g053(.a(new_n148), .b(new_n146), .c(new_n140), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n145), .b(new_n149), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .o1(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nano23aa1n02x4               g062(.a(new_n152), .b(new_n156), .c(new_n157), .d(new_n153), .out0(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  oai012aa1n02x5               g064(.a(new_n157), .b(new_n156), .c(new_n152), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n145), .d(new_n149), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  norp02aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoi112aa1n02x5               g072(.a(new_n167), .b(new_n163), .c(new_n161), .d(new_n164), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n167), .b(new_n163), .c(new_n161), .d(new_n164), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(\s[16] ));
  nano23aa1n02x4               g075(.a(new_n133), .b(new_n140), .c(new_n141), .d(new_n134), .out0(new_n171));
  nano23aa1n02x4               g076(.a(new_n163), .b(new_n165), .c(new_n166), .d(new_n164), .out0(new_n172));
  nanp02aa1n02x5               g077(.a(new_n172), .b(new_n158), .o1(new_n173));
  nano32aa1n02x4               g078(.a(new_n173), .b(new_n171), .c(new_n129), .d(new_n99), .out0(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n175));
  160nm_ficinv00aa1n08x5       g080(.clk(new_n147), .clkout(new_n176));
  nona22aa1n02x4               g081(.a(new_n171), .b(new_n176), .c(new_n138), .out0(new_n177));
  nona22aa1n02x4               g082(.a(new_n177), .b(new_n146), .c(new_n140), .out0(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n173), .clkout(new_n179));
  aoi112aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n160), .b(new_n172), .out0(new_n181));
  nona22aa1n02x4               g086(.a(new_n181), .b(new_n180), .c(new_n165), .out0(new_n182));
  aoi012aa1n02x5               g087(.a(new_n182), .b(new_n178), .c(new_n179), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n183), .b(new_n175), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[18] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[17] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\b[16] ), .clkout(new_n188));
  oaoi03aa1n02x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  xroi22aa1d04x5               g095(.a(new_n187), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n191));
  nanp02aa1n02x5               g096(.a(new_n188), .b(new_n187), .o1(new_n192));
  oaoi03aa1n02x5               g097(.a(\a[18] ), .b(\b[17] ), .c(new_n192), .o1(new_n193));
  norp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n193), .c(new_n184), .d(new_n191), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n196), .b(new_n193), .c(new_n184), .d(new_n191), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nona22aa1n02x4               g108(.a(new_n197), .b(new_n203), .c(new_n194), .out0(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n203), .clkout(new_n205));
  oaoi13aa1n02x5               g110(.a(new_n205), .b(new_n197), .c(\a[19] ), .d(\b[18] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n204), .b(new_n206), .out0(\s[20] ));
  nano23aa1n02x4               g112(.a(new_n194), .b(new_n201), .c(new_n202), .d(new_n195), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n191), .b(new_n208), .o1(new_n209));
  oai022aa1n02x5               g114(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n210));
  oaib12aa1n02x5               g115(.a(new_n210), .b(new_n186), .c(\b[17] ), .out0(new_n211));
  nona23aa1n02x4               g116(.a(new_n202), .b(new_n195), .c(new_n194), .d(new_n201), .out0(new_n212));
  aoi012aa1n02x5               g117(.a(new_n201), .b(new_n194), .c(new_n202), .o1(new_n213));
  oai012aa1n02x5               g118(.a(new_n213), .b(new_n212), .c(new_n211), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n209), .c(new_n183), .d(new_n175), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  xorc02aa1n02x5               g124(.a(\a[22] ), .b(\b[21] ), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n216), .d(new_n219), .o1(new_n221));
  aoai13aa1n02x5               g126(.a(new_n220), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[21] ), .clkout(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\a[22] ), .clkout(new_n225));
  xroi22aa1d04x5               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  nanp03aa1n02x5               g131(.a(new_n226), .b(new_n191), .c(new_n208), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(\b[21] ), .clkout(new_n228));
  oaoi03aa1n02x5               g133(.a(new_n225), .b(new_n228), .c(new_n218), .o1(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(new_n229), .clkout(new_n230));
  aoi012aa1n02x5               g135(.a(new_n230), .b(new_n214), .c(new_n226), .o1(new_n231));
  aoai13aa1n02x5               g136(.a(new_n231), .b(new_n227), .c(new_n183), .d(new_n175), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n02x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  xorc02aa1n02x5               g140(.a(\a[24] ), .b(\b[23] ), .out0(new_n236));
  aoi112aa1n02x5               g141(.a(new_n234), .b(new_n236), .c(new_n232), .d(new_n235), .o1(new_n237));
  aoai13aa1n02x5               g142(.a(new_n236), .b(new_n234), .c(new_n232), .d(new_n235), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(\s[24] ));
  and002aa1n02x5               g144(.a(new_n236), .b(new_n235), .o(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  nano32aa1n02x4               g146(.a(new_n241), .b(new_n226), .c(new_n191), .d(new_n208), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n213), .clkout(new_n243));
  aoai13aa1n02x5               g148(.a(new_n226), .b(new_n243), .c(new_n208), .d(new_n193), .o1(new_n244));
  orn002aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .o(new_n245));
  oao003aa1n02x5               g150(.a(\a[24] ), .b(\b[23] ), .c(new_n245), .carry(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n241), .c(new_n244), .d(new_n229), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n247), .c(new_n184), .d(new_n242), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n248), .b(new_n247), .c(new_n184), .d(new_n242), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[26] ), .b(\b[25] ), .out0(new_n253));
  nona22aa1n02x4               g158(.a(new_n249), .b(new_n253), .c(new_n252), .out0(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n252), .clkout(new_n255));
  aobi12aa1n02x5               g160(.a(new_n253), .b(new_n249), .c(new_n255), .out0(new_n256));
  norb02aa1n02x5               g161(.a(new_n254), .b(new_n256), .out0(\s[26] ));
  nanp02aa1n02x5               g162(.a(new_n112), .b(new_n120), .o1(new_n258));
  nanb02aa1n02x5               g163(.a(new_n126), .b(new_n258), .out0(new_n259));
  oabi12aa1n02x5               g164(.a(new_n182), .b(new_n149), .c(new_n173), .out0(new_n260));
  and002aa1n02x5               g165(.a(new_n253), .b(new_n248), .o(new_n261));
  nano22aa1n02x4               g166(.a(new_n227), .b(new_n240), .c(new_n261), .out0(new_n262));
  aoai13aa1n02x5               g167(.a(new_n262), .b(new_n260), .c(new_n259), .d(new_n174), .o1(new_n263));
  oao003aa1n02x5               g168(.a(\a[26] ), .b(\b[25] ), .c(new_n255), .carry(new_n264));
  aobi12aa1n02x5               g169(.a(new_n264), .b(new_n247), .c(new_n261), .out0(new_n265));
  xorc02aa1n02x5               g170(.a(\a[27] ), .b(\b[26] ), .out0(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n263), .out0(\s[27] ));
  norp02aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n268), .clkout(new_n269));
  aobi12aa1n02x5               g174(.a(new_n266), .b(new_n265), .c(new_n263), .out0(new_n270));
  xnrc02aa1n02x5               g175(.a(\b[27] ), .b(\a[28] ), .out0(new_n271));
  nano22aa1n02x4               g176(.a(new_n270), .b(new_n269), .c(new_n271), .out0(new_n272));
  aobi12aa1n02x5               g177(.a(new_n262), .b(new_n183), .c(new_n175), .out0(new_n273));
  aoai13aa1n02x5               g178(.a(new_n240), .b(new_n230), .c(new_n214), .d(new_n226), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n261), .clkout(new_n275));
  aoai13aa1n02x5               g180(.a(new_n264), .b(new_n275), .c(new_n274), .d(new_n246), .o1(new_n276));
  oai012aa1n02x5               g181(.a(new_n266), .b(new_n276), .c(new_n273), .o1(new_n277));
  aoi012aa1n02x5               g182(.a(new_n271), .b(new_n277), .c(new_n269), .o1(new_n278));
  norp02aa1n02x5               g183(.a(new_n278), .b(new_n272), .o1(\s[28] ));
  norb02aa1n02x5               g184(.a(new_n266), .b(new_n271), .out0(new_n280));
  oai012aa1n02x5               g185(.a(new_n280), .b(new_n276), .c(new_n273), .o1(new_n281));
  oao003aa1n02x5               g186(.a(\a[28] ), .b(\b[27] ), .c(new_n269), .carry(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[28] ), .b(\a[29] ), .out0(new_n283));
  aoi012aa1n02x5               g188(.a(new_n283), .b(new_n281), .c(new_n282), .o1(new_n284));
  aobi12aa1n02x5               g189(.a(new_n280), .b(new_n265), .c(new_n263), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n285), .b(new_n282), .c(new_n283), .out0(new_n286));
  norp02aa1n02x5               g191(.a(new_n284), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g193(.a(new_n266), .b(new_n283), .c(new_n271), .out0(new_n289));
  oai012aa1n02x5               g194(.a(new_n289), .b(new_n276), .c(new_n273), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[29] ), .b(\a[30] ), .out0(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  aobi12aa1n02x5               g198(.a(new_n289), .b(new_n265), .c(new_n263), .out0(new_n294));
  nano22aa1n02x4               g199(.a(new_n294), .b(new_n291), .c(new_n292), .out0(new_n295));
  norp02aa1n02x5               g200(.a(new_n293), .b(new_n295), .o1(\s[30] ));
  norb02aa1n02x5               g201(.a(new_n289), .b(new_n292), .out0(new_n297));
  aobi12aa1n02x5               g202(.a(new_n297), .b(new_n265), .c(new_n263), .out0(new_n298));
  oao003aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .c(new_n291), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  nano22aa1n02x4               g205(.a(new_n298), .b(new_n299), .c(new_n300), .out0(new_n301));
  oai012aa1n02x5               g206(.a(new_n297), .b(new_n276), .c(new_n273), .o1(new_n302));
  aoi012aa1n02x5               g207(.a(new_n300), .b(new_n302), .c(new_n299), .o1(new_n303));
  norp02aa1n02x5               g208(.a(new_n303), .b(new_n301), .o1(\s[31] ));
  xnbna2aa1n03x5               g209(.a(new_n110), .b(new_n104), .c(new_n105), .out0(\s[3] ));
  oaoi03aa1n02x5               g210(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g212(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g213(.a(\a[5] ), .b(\b[4] ), .o(new_n309));
  nanb02aa1n02x5               g214(.a(new_n119), .b(new_n112), .out0(new_n310));
  xobna2aa1n03x5               g215(.a(new_n118), .b(new_n310), .c(new_n309), .out0(\s[6] ));
  norp02aa1n02x5               g216(.a(new_n119), .b(new_n118), .o1(new_n312));
  aobi12aa1n02x5               g217(.a(new_n125), .b(new_n112), .c(new_n312), .out0(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g219(.a(\a[7] ), .b(\b[6] ), .c(new_n313), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g221(.a(new_n127), .b(new_n129), .out0(\s[9] ));
endmodule



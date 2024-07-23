// Benchmark "adder" written by ABC on Thu Jul 11 11:46:24 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n325, new_n328, new_n330, new_n332;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  and002aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\a[3] ), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\b[2] ), .clkout(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(new_n102), .b(new_n103), .o1(new_n104));
  160nm_ficinv00aa1n08x5       g009(.clk(\a[2] ), .clkout(new_n105));
  160nm_ficinv00aa1n08x5       g010(.clk(\b[1] ), .clkout(new_n106));
  nanp02aa1n02x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  aob012aa1n02x5               g013(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(new_n109));
  160nm_ficinv00aa1n08x5       g014(.clk(\a[4] ), .clkout(new_n110));
  aboi22aa1n03x5               g015(.a(\b[3] ), .b(new_n110), .c(new_n100), .d(new_n101), .out0(new_n111));
  aoai13aa1n02x5               g016(.a(new_n111), .b(new_n104), .c(new_n109), .d(new_n107), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  norp02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  norp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nano23aa1n02x4               g026(.a(new_n118), .b(new_n120), .c(new_n121), .d(new_n119), .out0(new_n122));
  nona23aa1n02x4               g027(.a(new_n112), .b(new_n122), .c(new_n117), .d(new_n99), .out0(new_n123));
  160nm_ficinv00aa1n08x5       g028(.clk(new_n113), .clkout(new_n124));
  nanp02aa1n02x5               g029(.a(new_n115), .b(new_n114), .o1(new_n125));
  oai012aa1n02x5               g030(.a(new_n119), .b(new_n120), .c(new_n118), .o1(new_n126));
  norp02aa1n02x5               g031(.a(new_n117), .b(new_n126), .o1(new_n127));
  nano22aa1n02x4               g032(.a(new_n127), .b(new_n124), .c(new_n125), .out0(new_n128));
  nanp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nanb02aa1n02x5               g034(.a(new_n97), .b(new_n129), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n98), .b(new_n130), .c(new_n123), .d(new_n128), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  oai012aa1n02x5               g039(.a(new_n134), .b(new_n97), .c(new_n133), .o1(new_n135));
  nona23aa1n02x4               g040(.a(new_n129), .b(new_n134), .c(new_n133), .d(new_n97), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n135), .b(new_n136), .c(new_n123), .d(new_n128), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  aoi012aa1n02x5               g045(.a(new_n139), .b(new_n137), .c(new_n140), .o1(new_n141));
  norp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  160nm_ficinv00aa1n08x5       g047(.clk(new_n142), .clkout(new_n143));
  nanp02aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n141), .b(new_n144), .c(new_n143), .out0(\s[12] ));
  aoi022aa1n02x5               g050(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n146));
  aoi012aa1n02x5               g051(.a(new_n146), .b(new_n105), .c(new_n106), .o1(new_n147));
  oaoi13aa1n02x5               g052(.a(new_n99), .b(new_n111), .c(new_n147), .d(new_n104), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n122), .b(new_n117), .out0(new_n149));
  oai112aa1n02x5               g054(.a(new_n124), .b(new_n125), .c(new_n117), .d(new_n126), .o1(new_n150));
  nona23aa1n02x4               g055(.a(new_n144), .b(new_n140), .c(new_n139), .d(new_n142), .out0(new_n151));
  norp02aa1n02x5               g056(.a(new_n151), .b(new_n136), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n150), .c(new_n148), .d(new_n149), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n139), .b(new_n144), .o1(new_n154));
  oai112aa1n02x5               g059(.a(new_n154), .b(new_n143), .c(new_n151), .d(new_n135), .o1(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  nanp02aa1n02x5               g061(.a(new_n153), .b(new_n156), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n159), .b(new_n157), .c(new_n160), .o1(new_n161));
  xnrb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nona23aa1n02x4               g069(.a(new_n164), .b(new_n160), .c(new_n159), .d(new_n163), .out0(new_n165));
  oai012aa1n02x5               g070(.a(new_n164), .b(new_n163), .c(new_n159), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n165), .c(new_n153), .d(new_n156), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  xorc02aa1n02x5               g074(.a(\a[15] ), .b(\b[14] ), .out0(new_n170));
  xorc02aa1n02x5               g075(.a(\a[16] ), .b(\b[15] ), .out0(new_n171));
  aoi112aa1n02x5               g076(.a(new_n171), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n171), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  nano23aa1n02x4               g079(.a(new_n133), .b(new_n97), .c(new_n129), .d(new_n134), .out0(new_n175));
  nano23aa1n02x4               g080(.a(new_n139), .b(new_n142), .c(new_n144), .d(new_n140), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n171), .b(new_n170), .o1(new_n177));
  nano23aa1n02x4               g082(.a(new_n177), .b(new_n165), .c(new_n176), .d(new_n175), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n150), .c(new_n148), .d(new_n149), .o1(new_n179));
  xnrc02aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .out0(new_n180));
  xnrc02aa1n02x5               g085(.a(\b[15] ), .b(\a[16] ), .out0(new_n181));
  norp03aa1n02x5               g086(.a(new_n165), .b(new_n181), .c(new_n180), .o1(new_n182));
  aoi112aa1n02x5               g087(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n183));
  oai022aa1n02x5               g088(.a(new_n177), .b(new_n166), .c(\b[15] ), .d(\a[16] ), .o1(new_n184));
  aoi112aa1n02x5               g089(.a(new_n184), .b(new_n183), .c(new_n155), .d(new_n182), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(new_n185), .b(new_n179), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[17] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\b[16] ), .clkout(new_n189));
  nanp02aa1n02x5               g094(.a(new_n189), .b(new_n188), .o1(new_n190));
  norp02aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  nanb02aa1n02x5               g097(.a(new_n191), .b(new_n192), .out0(new_n193));
  nanp02aa1n02x5               g098(.a(new_n182), .b(new_n152), .o1(new_n194));
  aoi012aa1n02x5               g099(.a(new_n194), .b(new_n123), .c(new_n128), .o1(new_n195));
  norp02aa1n02x5               g100(.a(\b[15] ), .b(\a[16] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(new_n155), .b(new_n182), .o1(new_n197));
  norp03aa1n02x5               g102(.a(new_n181), .b(new_n180), .c(new_n166), .o1(new_n198));
  nona32aa1n02x4               g103(.a(new_n197), .b(new_n198), .c(new_n183), .d(new_n196), .out0(new_n199));
  nanp02aa1n02x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  oai012aa1n02x5               g105(.a(new_n200), .b(new_n199), .c(new_n195), .o1(new_n201));
  xobna2aa1n03x5               g106(.a(new_n193), .b(new_n201), .c(new_n190), .out0(\s[18] ));
  nano22aa1n02x4               g107(.a(new_n193), .b(new_n190), .c(new_n200), .out0(new_n203));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n203), .clkout(new_n204));
  aoai13aa1n02x5               g109(.a(new_n192), .b(new_n191), .c(new_n188), .d(new_n189), .o1(new_n205));
  aoai13aa1n02x5               g110(.a(new_n205), .b(new_n204), .c(new_n185), .d(new_n179), .o1(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nanp02aa1n02x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(\b[19] ), .clkout(new_n213));
  nanb02aa1n02x5               g118(.a(\a[20] ), .b(new_n213), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(new_n214), .b(new_n215), .o1(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n216), .clkout(new_n217));
  aoi112aa1n02x5               g122(.a(new_n209), .b(new_n217), .c(new_n206), .d(new_n212), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n209), .clkout(new_n219));
  nanp02aa1n02x5               g124(.a(new_n206), .b(new_n212), .o1(new_n220));
  aoi012aa1n02x5               g125(.a(new_n216), .b(new_n220), .c(new_n219), .o1(new_n221));
  norp02aa1n02x5               g126(.a(new_n221), .b(new_n218), .o1(\s[20] ));
  nona22aa1n02x4               g127(.a(new_n203), .b(new_n211), .c(new_n216), .out0(new_n223));
  nanp02aa1n02x5               g128(.a(new_n209), .b(new_n215), .o1(new_n224));
  norp03aa1n02x5               g129(.a(new_n205), .b(new_n211), .c(new_n216), .o1(new_n225));
  nano22aa1n02x4               g130(.a(new_n225), .b(new_n214), .c(new_n224), .out0(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n223), .c(new_n185), .d(new_n179), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  xnrc02aa1n02x5               g134(.a(\b[20] ), .b(\a[21] ), .out0(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(new_n230), .clkout(new_n231));
  xnrc02aa1n02x5               g136(.a(\b[21] ), .b(\a[22] ), .out0(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  aoi112aa1n02x5               g138(.a(new_n229), .b(new_n233), .c(new_n227), .d(new_n231), .o1(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n229), .clkout(new_n235));
  nanp02aa1n02x5               g140(.a(new_n227), .b(new_n231), .o1(new_n236));
  aoi012aa1n02x5               g141(.a(new_n232), .b(new_n236), .c(new_n235), .o1(new_n237));
  norp02aa1n02x5               g142(.a(new_n237), .b(new_n234), .o1(\s[22] ));
  norp02aa1n02x5               g143(.a(\b[19] ), .b(\a[20] ), .o1(new_n239));
  nona23aa1n02x4               g144(.a(new_n215), .b(new_n210), .c(new_n209), .d(new_n239), .out0(new_n240));
  norp02aa1n02x5               g145(.a(new_n232), .b(new_n230), .o1(new_n241));
  nanb03aa1n02x5               g146(.a(new_n240), .b(new_n241), .c(new_n203), .out0(new_n242));
  oai112aa1n02x5               g147(.a(new_n224), .b(new_n214), .c(new_n240), .d(new_n205), .o1(new_n243));
  oaoi03aa1n02x5               g148(.a(\a[22] ), .b(\b[21] ), .c(new_n235), .o1(new_n244));
  aoi012aa1n02x5               g149(.a(new_n244), .b(new_n243), .c(new_n241), .o1(new_n245));
  aoai13aa1n02x5               g150(.a(new_n245), .b(new_n242), .c(new_n185), .d(new_n179), .o1(new_n246));
  xorb03aa1n02x5               g151(.a(new_n246), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  xorc02aa1n02x5               g153(.a(\a[23] ), .b(\b[22] ), .out0(new_n249));
  xorc02aa1n02x5               g154(.a(\a[24] ), .b(\b[23] ), .out0(new_n250));
  aoi112aa1n02x5               g155(.a(new_n248), .b(new_n250), .c(new_n246), .d(new_n249), .o1(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n248), .clkout(new_n252));
  nanp02aa1n02x5               g157(.a(new_n246), .b(new_n249), .o1(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n250), .clkout(new_n254));
  aoi012aa1n02x5               g159(.a(new_n254), .b(new_n253), .c(new_n252), .o1(new_n255));
  norp02aa1n02x5               g160(.a(new_n255), .b(new_n251), .o1(\s[24] ));
  nanp02aa1n02x5               g161(.a(new_n250), .b(new_n249), .o1(new_n257));
  nona23aa1n02x4               g162(.a(new_n241), .b(new_n203), .c(new_n257), .d(new_n240), .out0(new_n258));
  xnrc02aa1n02x5               g163(.a(\b[22] ), .b(\a[23] ), .out0(new_n259));
  norb02aa1n02x5               g164(.a(new_n250), .b(new_n259), .out0(new_n260));
  norp02aa1n02x5               g165(.a(\b[23] ), .b(\a[24] ), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n262));
  nanp03aa1n02x5               g167(.a(new_n244), .b(new_n249), .c(new_n250), .o1(new_n263));
  nona22aa1n02x4               g168(.a(new_n263), .b(new_n262), .c(new_n261), .out0(new_n264));
  aoi013aa1n02x4               g169(.a(new_n264), .b(new_n243), .c(new_n241), .d(new_n260), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n265), .b(new_n258), .c(new_n185), .d(new_n179), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  xorc02aa1n02x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xorc02aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  aoi112aa1n02x5               g175(.a(new_n268), .b(new_n270), .c(new_n266), .d(new_n269), .o1(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n268), .clkout(new_n272));
  nanp02aa1n02x5               g177(.a(new_n266), .b(new_n269), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n270), .clkout(new_n274));
  aoi012aa1n02x5               g179(.a(new_n274), .b(new_n273), .c(new_n272), .o1(new_n275));
  norp02aa1n02x5               g180(.a(new_n275), .b(new_n271), .o1(\s[26] ));
  and002aa1n02x5               g181(.a(new_n270), .b(new_n269), .o(new_n277));
  nano22aa1n02x4               g182(.a(new_n242), .b(new_n277), .c(new_n260), .out0(new_n278));
  oai012aa1n02x5               g183(.a(new_n278), .b(new_n199), .c(new_n195), .o1(new_n279));
  nano22aa1n02x4               g184(.a(new_n226), .b(new_n241), .c(new_n260), .out0(new_n280));
  oao003aa1n02x5               g185(.a(\a[26] ), .b(\b[25] ), .c(new_n272), .carry(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n281), .clkout(new_n282));
  oaoi13aa1n02x5               g187(.a(new_n282), .b(new_n277), .c(new_n280), .d(new_n264), .o1(new_n283));
  norp02aa1n02x5               g188(.a(\b[26] ), .b(\a[27] ), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(\b[26] ), .b(\a[27] ), .o1(new_n285));
  norb02aa1n02x5               g190(.a(new_n285), .b(new_n284), .out0(new_n286));
  xnbna2aa1n03x5               g191(.a(new_n286), .b(new_n279), .c(new_n283), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n284), .clkout(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[27] ), .b(\a[28] ), .out0(new_n289));
  nona32aa1n02x4               g194(.a(new_n243), .b(new_n257), .c(new_n232), .d(new_n230), .out0(new_n290));
  160nm_ficinv00aa1n08x5       g195(.clk(new_n264), .clkout(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(new_n277), .clkout(new_n292));
  aoai13aa1n02x5               g197(.a(new_n281), .b(new_n292), .c(new_n290), .d(new_n291), .o1(new_n293));
  aoai13aa1n02x5               g198(.a(new_n285), .b(new_n293), .c(new_n186), .d(new_n278), .o1(new_n294));
  aoi012aa1n02x5               g199(.a(new_n289), .b(new_n294), .c(new_n288), .o1(new_n295));
  aobi12aa1n02x5               g200(.a(new_n285), .b(new_n279), .c(new_n283), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n296), .b(new_n288), .c(new_n289), .out0(new_n297));
  norp02aa1n02x5               g202(.a(new_n295), .b(new_n297), .o1(\s[28] ));
  nano22aa1n02x4               g203(.a(new_n289), .b(new_n288), .c(new_n285), .out0(new_n299));
  aoai13aa1n02x5               g204(.a(new_n299), .b(new_n293), .c(new_n186), .d(new_n278), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[28] ), .b(\a[29] ), .out0(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n300), .c(new_n301), .o1(new_n303));
  aobi12aa1n02x5               g208(.a(new_n299), .b(new_n279), .c(new_n283), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n304), .b(new_n301), .c(new_n302), .out0(new_n305));
  norp02aa1n02x5               g210(.a(new_n303), .b(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g212(.a(new_n286), .b(new_n302), .c(new_n289), .out0(new_n308));
  aoai13aa1n02x5               g213(.a(new_n308), .b(new_n293), .c(new_n186), .d(new_n278), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[29] ), .b(\b[28] ), .c(new_n301), .carry(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[29] ), .b(\a[30] ), .out0(new_n311));
  aoi012aa1n02x5               g216(.a(new_n311), .b(new_n309), .c(new_n310), .o1(new_n312));
  aobi12aa1n02x5               g217(.a(new_n308), .b(new_n279), .c(new_n283), .out0(new_n313));
  nano22aa1n02x4               g218(.a(new_n313), .b(new_n310), .c(new_n311), .out0(new_n314));
  norp02aa1n02x5               g219(.a(new_n312), .b(new_n314), .o1(\s[30] ));
  norb03aa1n02x5               g220(.a(new_n299), .b(new_n311), .c(new_n302), .out0(new_n316));
  aobi12aa1n02x5               g221(.a(new_n316), .b(new_n279), .c(new_n283), .out0(new_n317));
  oao003aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .c(new_n310), .carry(new_n318));
  xnrc02aa1n02x5               g223(.a(\b[30] ), .b(\a[31] ), .out0(new_n319));
  nano22aa1n02x4               g224(.a(new_n317), .b(new_n318), .c(new_n319), .out0(new_n320));
  aoai13aa1n02x5               g225(.a(new_n316), .b(new_n293), .c(new_n186), .d(new_n278), .o1(new_n321));
  aoi012aa1n02x5               g226(.a(new_n319), .b(new_n321), .c(new_n318), .o1(new_n322));
  norp02aa1n02x5               g227(.a(new_n322), .b(new_n320), .o1(\s[31] ));
  xobna2aa1n03x5               g228(.a(new_n104), .b(new_n109), .c(new_n107), .out0(\s[3] ));
  aoai13aa1n02x5               g229(.a(new_n102), .b(new_n104), .c(new_n109), .d(new_n107), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g231(.a(new_n148), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g232(.a(new_n121), .b(new_n148), .c(new_n120), .o1(new_n328));
  xnrb03aa1n02x5               g233(.a(new_n328), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aob012aa1n02x5               g234(.a(new_n126), .b(new_n148), .c(new_n122), .out0(new_n330));
  xorb03aa1n02x5               g235(.a(new_n330), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g236(.a(new_n115), .b(new_n330), .c(new_n116), .o1(new_n332));
  xnbna2aa1n03x5               g237(.a(new_n332), .b(new_n124), .c(new_n114), .out0(\s[8] ));
  xobna2aa1n03x5               g238(.a(new_n130), .b(new_n123), .c(new_n128), .out0(\s[9] ));
endmodule


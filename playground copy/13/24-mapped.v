// Benchmark "adder" written by ABC on Thu Jul 11 11:51:55 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n228, new_n229, new_n230,
    new_n231, new_n232, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n302, new_n305,
    new_n306, new_n308, new_n310;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xnrc02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[9] ), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\b[8] ), .clkout(new_n99));
  nanp02aa1n02x5               g004(.a(new_n99), .b(new_n98), .o1(new_n100));
  and002aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\a[3] ), .clkout(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\b[2] ), .clkout(new_n103));
  nanp02aa1n02x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  aoi012aa1n02x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  160nm_ficinv00aa1n08x5       g015(.clk(\a[4] ), .clkout(new_n111));
  aboi22aa1n03x5               g016(.a(\b[3] ), .b(new_n111), .c(new_n102), .d(new_n103), .out0(new_n112));
  oaoi13aa1n02x5               g017(.a(new_n101), .b(new_n112), .c(new_n110), .d(new_n106), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nona23aa1n02x4               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  xnrc02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .out0(new_n120));
  norp03aa1n02x5               g025(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(\a[6] ), .clkout(new_n122));
  oai022aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  oaib12aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(\b[5] ), .out0(new_n124));
  160nm_fiao0012aa1n02p5x5     g029(.a(new_n114), .b(new_n116), .c(new_n115), .o(new_n125));
  oabi12aa1n02x5               g030(.a(new_n125), .b(new_n118), .c(new_n124), .out0(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n126), .c(new_n113), .d(new_n121), .o1(new_n128));
  xobna2aa1n03x5               g033(.a(new_n97), .b(new_n128), .c(new_n100), .out0(\s[10] ));
  nanp02aa1n02x5               g034(.a(new_n113), .b(new_n121), .o1(new_n130));
  160nm_ficinv00aa1n08x5       g035(.clk(new_n126), .clkout(new_n131));
  nanp02aa1n02x5               g036(.a(new_n130), .b(new_n131), .o1(new_n132));
  norp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  oaoi03aa1n02x5               g040(.a(\a[10] ), .b(\b[9] ), .c(new_n100), .o1(new_n136));
  nano22aa1n02x4               g041(.a(new_n97), .b(new_n100), .c(new_n127), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n135), .b(new_n136), .c(new_n132), .d(new_n137), .o1(new_n138));
  aoi112aa1n02x5               g043(.a(new_n136), .b(new_n135), .c(new_n132), .d(new_n137), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g045(.clk(new_n133), .clkout(new_n141));
  norp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n138), .c(new_n141), .out0(\s[12] ));
  norp02aa1n02x5               g050(.a(\b[12] ), .b(\a[13] ), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  nanb02aa1n02x5               g052(.a(new_n146), .b(new_n147), .out0(new_n148));
  nano23aa1n02x4               g053(.a(new_n133), .b(new_n142), .c(new_n143), .d(new_n134), .out0(new_n149));
  nano32aa1n02x4               g054(.a(new_n97), .b(new_n149), .c(new_n100), .d(new_n127), .out0(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n126), .c(new_n113), .d(new_n121), .o1(new_n151));
  oaoi03aa1n02x5               g056(.a(\a[12] ), .b(\b[11] ), .c(new_n141), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n152), .b(new_n149), .c(new_n136), .o1(new_n153));
  xobna2aa1n03x5               g058(.a(new_n148), .b(new_n151), .c(new_n153), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n146), .clkout(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n148), .c(new_n151), .d(new_n153), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nano23aa1n02x4               g064(.a(new_n146), .b(new_n158), .c(new_n159), .d(new_n147), .out0(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(new_n160), .clkout(new_n161));
  oai012aa1n02x5               g066(.a(new_n159), .b(new_n158), .c(new_n146), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n161), .c(new_n151), .d(new_n153), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  xorc02aa1n02x5               g070(.a(\a[15] ), .b(\b[14] ), .out0(new_n166));
  xorc02aa1n02x5               g071(.a(\a[16] ), .b(\b[15] ), .out0(new_n167));
  aoi112aa1n02x5               g072(.a(new_n167), .b(new_n165), .c(new_n163), .d(new_n166), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n167), .b(new_n165), .c(new_n163), .d(new_n166), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(\s[16] ));
  nanp03aa1n02x5               g075(.a(new_n160), .b(new_n166), .c(new_n167), .o1(new_n171));
  nano22aa1n02x4               g076(.a(new_n171), .b(new_n137), .c(new_n149), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n126), .c(new_n113), .d(new_n121), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(new_n167), .b(new_n166), .o1(new_n174));
  160nm_ficinv00aa1n08x5       g079(.clk(new_n165), .clkout(new_n175));
  oao003aa1n02x5               g080(.a(\a[16] ), .b(\b[15] ), .c(new_n175), .carry(new_n176));
  oai012aa1n02x5               g081(.a(new_n176), .b(new_n174), .c(new_n162), .o1(new_n177));
  oab012aa1n02x4               g082(.a(new_n177), .b(new_n153), .c(new_n171), .out0(new_n178));
  nanp02aa1n02x5               g083(.a(new_n173), .b(new_n178), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g085(.clk(\a[18] ), .clkout(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[17] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\b[16] ), .clkout(new_n183));
  oaoi03aa1n02x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  xroi22aa1d04x5               g090(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n183), .b(new_n182), .o1(new_n187));
  oaoi03aa1n02x5               g092(.a(\a[18] ), .b(\b[17] ), .c(new_n187), .o1(new_n188));
  norp02aa1n02x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  aoai13aa1n02x5               g096(.a(new_n191), .b(new_n188), .c(new_n179), .d(new_n186), .o1(new_n192));
  aoi112aa1n02x5               g097(.a(new_n191), .b(new_n188), .c(new_n179), .d(new_n186), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n192), .b(new_n193), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  nona22aa1n02x4               g103(.a(new_n192), .b(new_n198), .c(new_n189), .out0(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n198), .clkout(new_n200));
  oaoi13aa1n02x5               g105(.a(new_n200), .b(new_n192), .c(\a[19] ), .d(\b[18] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n199), .b(new_n201), .out0(\s[20] ));
  nano23aa1n02x4               g107(.a(new_n189), .b(new_n196), .c(new_n197), .d(new_n190), .out0(new_n203));
  nanp02aa1n02x5               g108(.a(new_n186), .b(new_n203), .o1(new_n204));
  oai022aa1n02x5               g109(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n205));
  oaib12aa1n02x5               g110(.a(new_n205), .b(new_n181), .c(\b[17] ), .out0(new_n206));
  nona23aa1n02x4               g111(.a(new_n197), .b(new_n190), .c(new_n189), .d(new_n196), .out0(new_n207));
  aoi012aa1n02x5               g112(.a(new_n196), .b(new_n189), .c(new_n197), .o1(new_n208));
  oai012aa1n02x5               g113(.a(new_n208), .b(new_n207), .c(new_n206), .o1(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n209), .clkout(new_n210));
  aoai13aa1n02x5               g115(.a(new_n210), .b(new_n204), .c(new_n173), .d(new_n178), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g117(.a(\b[20] ), .b(\a[21] ), .o1(new_n213));
  xorc02aa1n02x5               g118(.a(\a[21] ), .b(\b[20] ), .out0(new_n214));
  xorc02aa1n02x5               g119(.a(\a[22] ), .b(\b[21] ), .out0(new_n215));
  aoi112aa1n02x5               g120(.a(new_n213), .b(new_n215), .c(new_n211), .d(new_n214), .o1(new_n216));
  aoai13aa1n02x5               g121(.a(new_n215), .b(new_n213), .c(new_n211), .d(new_n214), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g123(.clk(\a[21] ), .clkout(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(\a[22] ), .clkout(new_n220));
  xroi22aa1d04x5               g125(.a(new_n219), .b(\b[20] ), .c(new_n220), .d(\b[21] ), .out0(new_n221));
  nanp03aa1n02x5               g126(.a(new_n221), .b(new_n186), .c(new_n203), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(\b[21] ), .clkout(new_n223));
  oao003aa1n02x5               g128(.a(new_n220), .b(new_n223), .c(new_n213), .carry(new_n224));
  aoi012aa1n02x5               g129(.a(new_n224), .b(new_n209), .c(new_n221), .o1(new_n225));
  aoai13aa1n02x5               g130(.a(new_n225), .b(new_n222), .c(new_n173), .d(new_n178), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g132(.a(\b[22] ), .b(\a[23] ), .o1(new_n228));
  xorc02aa1n02x5               g133(.a(\a[23] ), .b(\b[22] ), .out0(new_n229));
  xorc02aa1n02x5               g134(.a(\a[24] ), .b(\b[23] ), .out0(new_n230));
  aoi112aa1n02x5               g135(.a(new_n228), .b(new_n230), .c(new_n226), .d(new_n229), .o1(new_n231));
  aoai13aa1n02x5               g136(.a(new_n230), .b(new_n228), .c(new_n226), .d(new_n229), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n232), .b(new_n231), .out0(\s[24] ));
  and002aa1n02x5               g138(.a(new_n230), .b(new_n229), .o(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n234), .clkout(new_n235));
  nano32aa1n02x4               g140(.a(new_n235), .b(new_n221), .c(new_n186), .d(new_n203), .out0(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n208), .clkout(new_n237));
  aoai13aa1n02x5               g142(.a(new_n221), .b(new_n237), .c(new_n203), .d(new_n188), .o1(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n224), .clkout(new_n239));
  aoi112aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n240));
  oab012aa1n02x4               g145(.a(new_n240), .b(\a[24] ), .c(\b[23] ), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n235), .c(new_n238), .d(new_n239), .o1(new_n242));
  xorc02aa1n02x5               g147(.a(\a[25] ), .b(\b[24] ), .out0(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n242), .c(new_n179), .d(new_n236), .o1(new_n244));
  aoi112aa1n02x5               g149(.a(new_n243), .b(new_n242), .c(new_n179), .d(new_n236), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n244), .b(new_n245), .out0(\s[25] ));
  norp02aa1n02x5               g151(.a(\b[24] ), .b(\a[25] ), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[26] ), .b(\b[25] ), .out0(new_n248));
  nona22aa1n02x4               g153(.a(new_n244), .b(new_n248), .c(new_n247), .out0(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n247), .clkout(new_n250));
  aobi12aa1n02x5               g155(.a(new_n248), .b(new_n244), .c(new_n250), .out0(new_n251));
  norb02aa1n02x5               g156(.a(new_n249), .b(new_n251), .out0(\s[26] ));
  oai122aa1n02x7               g157(.a(new_n176), .b(new_n153), .c(new_n171), .d(new_n174), .e(new_n162), .o1(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(\a[25] ), .clkout(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(\a[26] ), .clkout(new_n255));
  xroi22aa1d04x5               g160(.a(new_n254), .b(\b[24] ), .c(new_n255), .d(\b[25] ), .out0(new_n256));
  nano22aa1n02x4               g161(.a(new_n222), .b(new_n234), .c(new_n256), .out0(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n253), .c(new_n132), .d(new_n172), .o1(new_n258));
  oao003aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .c(new_n250), .carry(new_n259));
  aobi12aa1n02x5               g164(.a(new_n259), .b(new_n242), .c(new_n256), .out0(new_n260));
  norp02aa1n02x5               g165(.a(\b[26] ), .b(\a[27] ), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(\b[26] ), .b(\a[27] ), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n260), .c(new_n258), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n261), .clkout(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[27] ), .b(\a[28] ), .out0(new_n266));
  aobi12aa1n02x5               g171(.a(new_n257), .b(new_n173), .c(new_n178), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n234), .b(new_n224), .c(new_n209), .d(new_n221), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n256), .clkout(new_n269));
  aoai13aa1n02x5               g174(.a(new_n259), .b(new_n269), .c(new_n268), .d(new_n241), .o1(new_n270));
  oai012aa1n02x5               g175(.a(new_n262), .b(new_n270), .c(new_n267), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n266), .b(new_n271), .c(new_n265), .o1(new_n272));
  aoi022aa1n02x5               g177(.a(new_n260), .b(new_n258), .c(\a[27] ), .d(\b[26] ), .o1(new_n273));
  nano22aa1n02x4               g178(.a(new_n273), .b(new_n265), .c(new_n266), .out0(new_n274));
  norp02aa1n02x5               g179(.a(new_n272), .b(new_n274), .o1(\s[28] ));
  nano22aa1n02x4               g180(.a(new_n266), .b(new_n265), .c(new_n262), .out0(new_n276));
  oai012aa1n02x5               g181(.a(new_n276), .b(new_n270), .c(new_n267), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[28] ), .b(\b[27] ), .c(new_n265), .carry(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[28] ), .b(\a[29] ), .out0(new_n279));
  aoi012aa1n02x5               g184(.a(new_n279), .b(new_n277), .c(new_n278), .o1(new_n280));
  aobi12aa1n02x5               g185(.a(new_n276), .b(new_n260), .c(new_n258), .out0(new_n281));
  nano22aa1n02x4               g186(.a(new_n281), .b(new_n278), .c(new_n279), .out0(new_n282));
  norp02aa1n02x5               g187(.a(new_n280), .b(new_n282), .o1(\s[29] ));
  xorb03aa1n02x5               g188(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g189(.a(new_n263), .b(new_n279), .c(new_n266), .out0(new_n285));
  oai012aa1n02x5               g190(.a(new_n285), .b(new_n270), .c(new_n267), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[29] ), .b(\b[28] ), .c(new_n278), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[29] ), .b(\a[30] ), .out0(new_n288));
  aoi012aa1n02x5               g193(.a(new_n288), .b(new_n286), .c(new_n287), .o1(new_n289));
  aobi12aa1n02x5               g194(.a(new_n285), .b(new_n260), .c(new_n258), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n287), .c(new_n288), .out0(new_n291));
  norp02aa1n02x5               g196(.a(new_n289), .b(new_n291), .o1(\s[30] ));
  xnrc02aa1n02x5               g197(.a(\b[30] ), .b(\a[31] ), .out0(new_n293));
  norb03aa1n02x5               g198(.a(new_n276), .b(new_n288), .c(new_n279), .out0(new_n294));
  aobi12aa1n02x5               g199(.a(new_n294), .b(new_n260), .c(new_n258), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[30] ), .b(\b[29] ), .c(new_n287), .carry(new_n296));
  nano22aa1n02x4               g201(.a(new_n295), .b(new_n293), .c(new_n296), .out0(new_n297));
  oai012aa1n02x5               g202(.a(new_n294), .b(new_n270), .c(new_n267), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n293), .b(new_n298), .c(new_n296), .o1(new_n299));
  norp02aa1n02x5               g204(.a(new_n299), .b(new_n297), .o1(\s[31] ));
  xnbna2aa1n03x5               g205(.a(new_n110), .b(new_n104), .c(new_n105), .out0(\s[3] ));
  oaoi03aa1n02x5               g206(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n302));
  xorb03aa1n02x5               g207(.a(new_n302), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g208(.a(new_n113), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  160nm_ficinv00aa1n08x5       g209(.clk(new_n113), .clkout(new_n305));
  oaoi03aa1n02x5               g210(.a(\a[5] ), .b(\b[4] ), .c(new_n305), .o1(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai013aa1n02x4               g212(.a(new_n124), .b(new_n305), .c(new_n119), .d(new_n120), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g214(.a(new_n116), .b(new_n308), .c(new_n117), .o1(new_n310));
  xnrb03aa1n02x5               g215(.a(new_n310), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g216(.a(new_n132), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule



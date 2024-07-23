// Benchmark "adder" written by ABC on Thu Jul 11 12:11:58 2024

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
    new_n140, new_n141, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n342, new_n345, new_n347, new_n349;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[9] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[8] ), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(new_n102), .clkout(new_n103));
  norp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nano23aa1n02x4               g012(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n108));
  aoi012aa1n02x5               g013(.a(new_n104), .b(new_n106), .c(new_n105), .o1(new_n109));
  160nm_ficinv00aa1n08x5       g014(.clk(new_n109), .clkout(new_n110));
  aoi012aa1n02x5               g015(.a(new_n110), .b(new_n108), .c(new_n103), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n02x4               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  norp02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nanb02aa1n02x5               g023(.a(new_n117), .b(new_n118), .out0(new_n119));
  orn002aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .o(new_n120));
  nanp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n120), .b(new_n121), .o1(new_n122));
  nona22aa1n02x4               g027(.a(new_n116), .b(new_n122), .c(new_n119), .out0(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n124));
  160nm_fiao0012aa1n02p5x5     g029(.a(new_n112), .b(new_n114), .c(new_n113), .o(new_n125));
  aoi012aa1n02x5               g030(.a(new_n125), .b(new_n116), .c(new_n124), .o1(new_n126));
  oai012aa1n02x5               g031(.a(new_n126), .b(new_n111), .c(new_n123), .o1(new_n127));
  oaoi03aa1n02x5               g032(.a(new_n97), .b(new_n98), .c(new_n127), .o1(new_n128));
  xnrb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  norp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n133), .c(new_n97), .d(new_n98), .o1(new_n135));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n135), .clkout(new_n136));
  norp02aa1n02x5               g041(.a(\b[8] ), .b(\a[9] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[8] ), .b(\a[9] ), .o1(new_n138));
  nano23aa1n02x4               g043(.a(new_n133), .b(new_n137), .c(new_n138), .d(new_n134), .out0(new_n139));
  aoai13aa1n02x5               g044(.a(new_n132), .b(new_n136), .c(new_n127), .d(new_n139), .o1(new_n140));
  aoi112aa1n02x5               g045(.a(new_n136), .b(new_n132), .c(new_n127), .d(new_n139), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(\s[11] ));
  oai012aa1n02x5               g047(.a(new_n140), .b(\b[10] ), .c(\a[11] ), .o1(new_n143));
  xorb03aa1n02x5               g048(.a(new_n143), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nanb02aa1n02x5               g049(.a(new_n104), .b(new_n105), .out0(new_n145));
  160nm_ficinv00aa1n08x5       g050(.clk(\a[3] ), .clkout(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(\b[2] ), .clkout(new_n147));
  nanp02aa1n02x5               g052(.a(new_n147), .b(new_n146), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n148), .b(new_n107), .o1(new_n149));
  norp03aa1n02x5               g054(.a(new_n102), .b(new_n145), .c(new_n149), .o1(new_n150));
  nona23aa1n02x4               g055(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n151));
  norp03aa1n02x5               g056(.a(new_n151), .b(new_n119), .c(new_n122), .o1(new_n152));
  oai012aa1n02x5               g057(.a(new_n152), .b(new_n150), .c(new_n110), .o1(new_n153));
  norp02aa1n02x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  nano23aa1n02x4               g060(.a(new_n130), .b(new_n154), .c(new_n155), .d(new_n131), .out0(new_n156));
  nanp02aa1n02x5               g061(.a(new_n156), .b(new_n139), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n154), .b(new_n130), .c(new_n155), .o1(new_n158));
  aobi12aa1n02x5               g063(.a(new_n158), .b(new_n156), .c(new_n136), .out0(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n157), .c(new_n153), .d(new_n126), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g066(.clk(\a[13] ), .clkout(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(\b[12] ), .clkout(new_n163));
  oaoi03aa1n02x5               g068(.a(new_n162), .b(new_n163), .c(new_n160), .o1(new_n164));
  xnrb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oai013aa1n02x4               g070(.a(new_n109), .b(new_n145), .c(new_n102), .d(new_n149), .o1(new_n166));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n124), .clkout(new_n167));
  oabi12aa1n02x5               g072(.a(new_n125), .b(new_n167), .c(new_n151), .out0(new_n168));
  160nm_ficinv00aa1n08x5       g073(.clk(new_n157), .clkout(new_n169));
  aoai13aa1n02x5               g074(.a(new_n169), .b(new_n168), .c(new_n166), .d(new_n152), .o1(new_n170));
  norp02aa1n02x5               g075(.a(\b[12] ), .b(\a[13] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[12] ), .b(\a[13] ), .o1(new_n172));
  norp02aa1n02x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nona23aa1n02x4               g079(.a(new_n174), .b(new_n172), .c(new_n171), .d(new_n173), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n174), .b(new_n173), .c(new_n162), .d(new_n163), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n175), .c(new_n170), .d(new_n159), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(new_n181), .clkout(new_n182));
  norp02aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanb02aa1n02x5               g089(.a(new_n183), .b(new_n184), .out0(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(new_n185), .clkout(new_n186));
  aoi112aa1n02x5               g091(.a(new_n186), .b(new_n179), .c(new_n177), .d(new_n182), .o1(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(new_n179), .clkout(new_n188));
  nano23aa1n02x4               g093(.a(new_n171), .b(new_n173), .c(new_n174), .d(new_n172), .out0(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n176), .clkout(new_n190));
  aoai13aa1n02x5               g095(.a(new_n182), .b(new_n190), .c(new_n160), .d(new_n189), .o1(new_n191));
  aoi012aa1n02x5               g096(.a(new_n185), .b(new_n191), .c(new_n188), .o1(new_n192));
  norp02aa1n02x5               g097(.a(new_n192), .b(new_n187), .o1(\s[16] ));
  nona23aa1n02x4               g098(.a(new_n155), .b(new_n131), .c(new_n130), .d(new_n154), .out0(new_n194));
  nona23aa1n02x4               g099(.a(new_n184), .b(new_n180), .c(new_n179), .d(new_n183), .out0(new_n195));
  nona23aa1n02x4               g100(.a(new_n189), .b(new_n139), .c(new_n195), .d(new_n194), .out0(new_n196));
  oai012aa1n02x5               g101(.a(new_n158), .b(new_n194), .c(new_n135), .o1(new_n197));
  norp02aa1n02x5               g102(.a(new_n195), .b(new_n175), .o1(new_n198));
  aoi012aa1n02x5               g103(.a(new_n183), .b(new_n179), .c(new_n184), .o1(new_n199));
  oai012aa1n02x5               g104(.a(new_n199), .b(new_n195), .c(new_n176), .o1(new_n200));
  aoi012aa1n02x5               g105(.a(new_n200), .b(new_n197), .c(new_n198), .o1(new_n201));
  aoai13aa1n02x5               g106(.a(new_n201), .b(new_n196), .c(new_n153), .d(new_n126), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g108(.clk(\a[18] ), .clkout(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(\a[17] ), .clkout(new_n205));
  160nm_ficinv00aa1n08x5       g110(.clk(\b[16] ), .clkout(new_n206));
  oaoi03aa1n02x5               g111(.a(new_n205), .b(new_n206), .c(new_n202), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[17] ), .c(new_n204), .out0(\s[18] ));
  nona22aa1n02x4               g113(.a(new_n189), .b(new_n185), .c(new_n181), .out0(new_n209));
  norp02aa1n02x5               g114(.a(new_n209), .b(new_n157), .o1(new_n210));
  aoai13aa1n02x5               g115(.a(new_n210), .b(new_n168), .c(new_n166), .d(new_n152), .o1(new_n211));
  xroi22aa1d04x5               g116(.a(new_n205), .b(\b[16] ), .c(new_n204), .d(\b[17] ), .out0(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  nanp02aa1n02x5               g118(.a(new_n206), .b(new_n205), .o1(new_n214));
  oaoi03aa1n02x5               g119(.a(\a[18] ), .b(\b[17] ), .c(new_n214), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n213), .c(new_n211), .d(new_n201), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  norp02aa1n02x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nanb02aa1n02x5               g129(.a(new_n223), .b(new_n224), .out0(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n225), .clkout(new_n226));
  aoi112aa1n02x5               g131(.a(new_n220), .b(new_n226), .c(new_n217), .d(new_n222), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n220), .clkout(new_n228));
  aoai13aa1n02x5               g133(.a(new_n222), .b(new_n215), .c(new_n202), .d(new_n212), .o1(new_n229));
  aoi012aa1n02x5               g134(.a(new_n225), .b(new_n229), .c(new_n228), .o1(new_n230));
  norp02aa1n02x5               g135(.a(new_n230), .b(new_n227), .o1(\s[20] ));
  nano23aa1n02x4               g136(.a(new_n220), .b(new_n223), .c(new_n224), .d(new_n221), .out0(new_n232));
  nanp02aa1n02x5               g137(.a(new_n212), .b(new_n232), .o1(new_n233));
  nanp02aa1n02x5               g138(.a(new_n232), .b(new_n215), .o1(new_n234));
  aoi012aa1n02x5               g139(.a(new_n223), .b(new_n220), .c(new_n224), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(new_n234), .b(new_n235), .o1(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n236), .clkout(new_n237));
  aoai13aa1n02x5               g142(.a(new_n237), .b(new_n233), .c(new_n211), .d(new_n201), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  nanb02aa1n02x5               g146(.a(new_n240), .b(new_n241), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n242), .clkout(new_n243));
  norp02aa1n02x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  nanp02aa1n02x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  nanb02aa1n02x5               g150(.a(new_n244), .b(new_n245), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n246), .clkout(new_n247));
  aoi112aa1n02x5               g152(.a(new_n240), .b(new_n247), .c(new_n238), .d(new_n243), .o1(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n240), .clkout(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n233), .clkout(new_n250));
  aoai13aa1n02x5               g155(.a(new_n243), .b(new_n236), .c(new_n202), .d(new_n250), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n246), .b(new_n251), .c(new_n249), .o1(new_n252));
  norp02aa1n02x5               g157(.a(new_n252), .b(new_n248), .o1(\s[22] ));
  nano23aa1n02x4               g158(.a(new_n240), .b(new_n244), .c(new_n245), .d(new_n241), .out0(new_n254));
  nano22aa1n02x4               g159(.a(new_n213), .b(new_n232), .c(new_n254), .out0(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n255), .clkout(new_n256));
  oaoi03aa1n02x5               g161(.a(\a[22] ), .b(\b[21] ), .c(new_n249), .o1(new_n257));
  aoi012aa1n02x5               g162(.a(new_n257), .b(new_n236), .c(new_n254), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n256), .c(new_n211), .d(new_n201), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[23] ), .b(\b[22] ), .out0(new_n262));
  xorc02aa1n02x5               g167(.a(\a[24] ), .b(\b[23] ), .out0(new_n263));
  aoi112aa1n02x5               g168(.a(new_n261), .b(new_n263), .c(new_n259), .d(new_n262), .o1(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n261), .clkout(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n258), .clkout(new_n266));
  aoai13aa1n02x5               g171(.a(new_n262), .b(new_n266), .c(new_n202), .d(new_n255), .o1(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[23] ), .b(\a[24] ), .out0(new_n268));
  aoi012aa1n02x5               g173(.a(new_n268), .b(new_n267), .c(new_n265), .o1(new_n269));
  norp02aa1n02x5               g174(.a(new_n269), .b(new_n264), .o1(\s[24] ));
  nanp03aa1n02x5               g175(.a(new_n254), .b(new_n262), .c(new_n263), .o1(new_n271));
  nano22aa1n02x4               g176(.a(new_n271), .b(new_n212), .c(new_n232), .out0(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n272), .clkout(new_n273));
  oaoi03aa1n02x5               g178(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .o1(new_n274));
  aoi013aa1n02x4               g179(.a(new_n274), .b(new_n257), .c(new_n262), .d(new_n263), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n275), .b(new_n271), .c(new_n234), .d(new_n235), .o1(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n276), .clkout(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n273), .c(new_n211), .d(new_n201), .o1(new_n278));
  xorb03aa1n02x5               g183(.a(new_n278), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  xorc02aa1n02x5               g186(.a(\a[26] ), .b(\b[25] ), .out0(new_n282));
  aoi112aa1n02x5               g187(.a(new_n280), .b(new_n282), .c(new_n278), .d(new_n281), .o1(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(new_n280), .clkout(new_n284));
  aoai13aa1n02x5               g189(.a(new_n281), .b(new_n276), .c(new_n202), .d(new_n272), .o1(new_n285));
  160nm_ficinv00aa1n08x5       g190(.clk(new_n282), .clkout(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n285), .c(new_n284), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n287), .b(new_n283), .o1(\s[26] ));
  oabi12aa1n02x5               g193(.a(new_n200), .b(new_n159), .c(new_n209), .out0(new_n289));
  nona23aa1n02x4               g194(.a(new_n245), .b(new_n241), .c(new_n240), .d(new_n244), .out0(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[22] ), .b(\a[23] ), .out0(new_n291));
  norp03aa1n02x5               g196(.a(new_n290), .b(new_n291), .c(new_n268), .o1(new_n292));
  and002aa1n02x5               g197(.a(new_n282), .b(new_n281), .o(new_n293));
  nano22aa1n02x4               g198(.a(new_n233), .b(new_n292), .c(new_n293), .out0(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n289), .c(new_n127), .d(new_n210), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[26] ), .b(\b[25] ), .c(new_n284), .carry(new_n296));
  aobi12aa1n02x5               g201(.a(new_n296), .b(new_n276), .c(new_n293), .out0(new_n297));
  norp02aa1n02x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  nanp02aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .o1(new_n299));
  nanb02aa1n02x5               g204(.a(new_n298), .b(new_n299), .out0(new_n300));
  xobna2aa1n03x5               g205(.a(new_n300), .b(new_n295), .c(new_n297), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g206(.clk(new_n298), .clkout(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[27] ), .b(\a[28] ), .out0(new_n303));
  160nm_ficinv00aa1n08x5       g208(.clk(new_n235), .clkout(new_n304));
  aoai13aa1n02x5               g209(.a(new_n292), .b(new_n304), .c(new_n215), .d(new_n232), .o1(new_n305));
  160nm_ficinv00aa1n08x5       g210(.clk(new_n293), .clkout(new_n306));
  aoai13aa1n02x5               g211(.a(new_n296), .b(new_n306), .c(new_n305), .d(new_n275), .o1(new_n307));
  aoai13aa1n02x5               g212(.a(new_n299), .b(new_n307), .c(new_n202), .d(new_n294), .o1(new_n308));
  aoi012aa1n02x5               g213(.a(new_n303), .b(new_n308), .c(new_n302), .o1(new_n309));
  aoi022aa1n02x5               g214(.a(new_n295), .b(new_n297), .c(\b[26] ), .d(\a[27] ), .o1(new_n310));
  nano22aa1n02x4               g215(.a(new_n310), .b(new_n302), .c(new_n303), .out0(new_n311));
  norp02aa1n02x5               g216(.a(new_n309), .b(new_n311), .o1(\s[28] ));
  norp02aa1n02x5               g217(.a(new_n303), .b(new_n300), .o1(new_n313));
  aoai13aa1n02x5               g218(.a(new_n313), .b(new_n307), .c(new_n202), .d(new_n294), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .c(new_n302), .carry(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[28] ), .b(\a[29] ), .out0(new_n316));
  aoi012aa1n02x5               g221(.a(new_n316), .b(new_n314), .c(new_n315), .o1(new_n317));
  160nm_ficinv00aa1n08x5       g222(.clk(new_n313), .clkout(new_n318));
  aoi012aa1n02x5               g223(.a(new_n318), .b(new_n295), .c(new_n297), .o1(new_n319));
  nano22aa1n02x4               g224(.a(new_n319), .b(new_n315), .c(new_n316), .out0(new_n320));
  norp02aa1n02x5               g225(.a(new_n317), .b(new_n320), .o1(\s[29] ));
  xorb03aa1n02x5               g226(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norp03aa1n02x5               g227(.a(new_n316), .b(new_n303), .c(new_n300), .o1(new_n323));
  aoai13aa1n02x5               g228(.a(new_n323), .b(new_n307), .c(new_n202), .d(new_n294), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .carry(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[29] ), .b(\a[30] ), .out0(new_n326));
  aoi012aa1n02x5               g231(.a(new_n326), .b(new_n324), .c(new_n325), .o1(new_n327));
  160nm_ficinv00aa1n08x5       g232(.clk(new_n323), .clkout(new_n328));
  aoi012aa1n02x5               g233(.a(new_n328), .b(new_n295), .c(new_n297), .o1(new_n329));
  nano22aa1n02x4               g234(.a(new_n329), .b(new_n325), .c(new_n326), .out0(new_n330));
  norp02aa1n02x5               g235(.a(new_n327), .b(new_n330), .o1(\s[30] ));
  xnrc02aa1n02x5               g236(.a(\b[30] ), .b(\a[31] ), .out0(new_n332));
  norb03aa1n02x5               g237(.a(new_n313), .b(new_n326), .c(new_n316), .out0(new_n333));
  160nm_ficinv00aa1n08x5       g238(.clk(new_n333), .clkout(new_n334));
  aoi012aa1n02x5               g239(.a(new_n334), .b(new_n295), .c(new_n297), .o1(new_n335));
  oao003aa1n02x5               g240(.a(\a[30] ), .b(\b[29] ), .c(new_n325), .carry(new_n336));
  nano22aa1n02x4               g241(.a(new_n335), .b(new_n332), .c(new_n336), .out0(new_n337));
  aoai13aa1n02x5               g242(.a(new_n333), .b(new_n307), .c(new_n202), .d(new_n294), .o1(new_n338));
  aoi012aa1n02x5               g243(.a(new_n332), .b(new_n338), .c(new_n336), .o1(new_n339));
  norp02aa1n02x5               g244(.a(new_n339), .b(new_n337), .o1(\s[31] ));
  xnbna2aa1n03x5               g245(.a(new_n102), .b(new_n107), .c(new_n148), .out0(\s[3] ));
  oaoi03aa1n02x5               g246(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g248(.a(new_n166), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoai13aa1n02x5               g249(.a(new_n121), .b(new_n110), .c(new_n108), .d(new_n103), .o1(new_n345));
  xobna2aa1n03x5               g250(.a(new_n119), .b(new_n345), .c(new_n120), .out0(\s[6] ));
  oai013aa1n02x4               g251(.a(new_n167), .b(new_n111), .c(new_n119), .d(new_n122), .o1(new_n347));
  xorb03aa1n02x5               g252(.a(new_n347), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g253(.a(new_n114), .b(new_n347), .c(new_n115), .o1(new_n349));
  xnrb03aa1n02x5               g254(.a(new_n349), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g255(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule



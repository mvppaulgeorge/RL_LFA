// Benchmark "adder" written by ABC on Thu Jul 11 12:52:25 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n179, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n189, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n307, new_n309, new_n311, new_n313;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[9] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[8] ), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano23aa1n02x4               g008(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n104));
  nanp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  oa0012aa1n02x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o(new_n108));
  oa0012aa1n02x5               g013(.a(new_n101), .b(new_n102), .c(new_n100), .o(new_n109));
  aoi012aa1n02x5               g014(.a(new_n109), .b(new_n104), .c(new_n108), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  160nm_ficinv00aa1n08x5       g016(.clk(new_n111), .clkout(new_n112));
  oao003aa1n02x5               g017(.a(\a[4] ), .b(\b[3] ), .c(new_n112), .carry(new_n113));
  xorc02aa1n02x5               g018(.a(\a[4] ), .b(\b[3] ), .out0(new_n114));
  nanp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  norb02aa1n02x5               g020(.a(new_n115), .b(new_n111), .out0(new_n116));
  and002aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o(new_n117));
  nanp02aa1n02x5               g022(.a(\b[0] ), .b(\a[1] ), .o1(new_n118));
  norp02aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  oab012aa1n02x4               g024(.a(new_n117), .b(new_n119), .c(new_n118), .out0(new_n120));
  nanp03aa1n02x5               g025(.a(new_n120), .b(new_n114), .c(new_n116), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  nano23aa1n02x4               g027(.a(new_n107), .b(new_n106), .c(new_n122), .d(new_n105), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(new_n123), .b(new_n104), .o1(new_n124));
  aoai13aa1n02x5               g029(.a(new_n110), .b(new_n124), .c(new_n121), .d(new_n113), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  nanp02aa1n02x5               g031(.a(new_n125), .b(new_n126), .o1(new_n127));
  norp02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nanb02aa1n02x5               g034(.a(new_n128), .b(new_n129), .out0(new_n130));
  xobna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  norb02aa1n02x5               g036(.a(new_n126), .b(new_n130), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n129), .b(new_n128), .c(new_n97), .d(new_n98), .o1(new_n133));
  aob012aa1n02x5               g038(.a(new_n133), .b(new_n125), .c(new_n132), .out0(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  aoi012aa1n02x5               g042(.a(new_n136), .b(new_n134), .c(new_n137), .o1(new_n138));
  xnrb03aa1n02x5               g043(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nona23aa1n02x4               g046(.a(new_n141), .b(new_n137), .c(new_n136), .d(new_n140), .out0(new_n142));
  oa0012aa1n02x5               g047(.a(new_n141), .b(new_n140), .c(new_n136), .o(new_n143));
  oabi12aa1n02x5               g048(.a(new_n143), .b(new_n133), .c(new_n142), .out0(new_n144));
  nano23aa1n02x4               g049(.a(new_n136), .b(new_n140), .c(new_n141), .d(new_n137), .out0(new_n145));
  nanb03aa1n02x5               g050(.a(new_n130), .b(new_n145), .c(new_n126), .out0(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n146), .clkout(new_n147));
  xnrc02aa1n02x5               g052(.a(\b[12] ), .b(\a[13] ), .out0(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n144), .c(new_n125), .d(new_n147), .o1(new_n150));
  aoi112aa1n02x5               g055(.a(new_n144), .b(new_n149), .c(new_n125), .d(new_n147), .o1(new_n151));
  norb02aa1n02x5               g056(.a(new_n150), .b(new_n151), .out0(\s[13] ));
  norp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  160nm_ficinv00aa1n08x5       g058(.clk(new_n153), .clkout(new_n154));
  xnrc02aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .out0(new_n155));
  xobna2aa1n03x5               g060(.a(new_n155), .b(new_n150), .c(new_n154), .out0(\s[14] ));
  norp02aa1n02x5               g061(.a(new_n155), .b(new_n148), .o1(new_n157));
  oaoi03aa1n02x5               g062(.a(\a[14] ), .b(\b[13] ), .c(new_n154), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n158), .b(new_n144), .c(new_n157), .o1(new_n159));
  nona32aa1n02x4               g064(.a(new_n125), .b(new_n155), .c(new_n148), .d(new_n146), .out0(new_n160));
  xnrc02aa1n02x5               g065(.a(\b[14] ), .b(\a[15] ), .out0(new_n161));
  xobna2aa1n03x5               g066(.a(new_n161), .b(new_n160), .c(new_n159), .out0(\s[15] ));
  norp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  160nm_ficinv00aa1n08x5       g068(.clk(new_n163), .clkout(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n161), .c(new_n160), .d(new_n159), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  160nm_ficinv00aa1n08x5       g071(.clk(\a[17] ), .clkout(new_n167));
  xnrc02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .out0(new_n168));
  norp02aa1n02x5               g073(.a(new_n168), .b(new_n161), .o1(new_n169));
  nano22aa1n02x4               g074(.a(new_n146), .b(new_n157), .c(new_n169), .out0(new_n170));
  oaoi03aa1n02x5               g075(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n157), .b(new_n143), .c(new_n145), .d(new_n171), .o1(new_n172));
  160nm_ficinv00aa1n08x5       g077(.clk(new_n158), .clkout(new_n173));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n169), .clkout(new_n174));
  aoi012aa1n02x5               g079(.a(new_n174), .b(new_n172), .c(new_n173), .o1(new_n175));
  oaoi03aa1n02x5               g080(.a(\a[16] ), .b(\b[15] ), .c(new_n164), .o1(new_n176));
  aoi112aa1n02x5               g081(.a(new_n175), .b(new_n176), .c(new_n125), .d(new_n170), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[16] ), .c(new_n167), .out0(\s[17] ));
  oaoi03aa1n02x5               g083(.a(\a[17] ), .b(\b[16] ), .c(new_n177), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g085(.clk(\a[18] ), .clkout(new_n181));
  xroi22aa1d04x5               g086(.a(new_n167), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n182), .clkout(new_n183));
  oai022aa1n02x5               g088(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n184));
  oaib12aa1n02x5               g089(.a(new_n184), .b(new_n181), .c(\b[17] ), .out0(new_n185));
  oai012aa1n02x5               g090(.a(new_n185), .b(new_n177), .c(new_n183), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g092(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(new_n189), .b(new_n190), .out0(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(new_n191), .clkout(new_n192));
  norp02aa1n02x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n189), .c(new_n186), .d(new_n192), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(new_n125), .b(new_n170), .o1(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(new_n176), .clkout(new_n198));
  oai112aa1n02x5               g103(.a(new_n197), .b(new_n198), .c(new_n159), .d(new_n174), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(\b[16] ), .b(new_n167), .out0(new_n200));
  oaoi03aa1n02x5               g105(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n201));
  aoai13aa1n02x5               g106(.a(new_n192), .b(new_n201), .c(new_n199), .d(new_n182), .o1(new_n202));
  nona22aa1n02x4               g107(.a(new_n202), .b(new_n195), .c(new_n189), .out0(new_n203));
  nanp02aa1n02x5               g108(.a(new_n196), .b(new_n203), .o1(\s[20] ));
  nona23aa1n02x4               g109(.a(new_n194), .b(new_n190), .c(new_n189), .d(new_n193), .out0(new_n205));
  oa0012aa1n02x5               g110(.a(new_n194), .b(new_n193), .c(new_n189), .o(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n206), .clkout(new_n207));
  oai012aa1n02x5               g112(.a(new_n207), .b(new_n205), .c(new_n185), .o1(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n208), .clkout(new_n209));
  nano23aa1n02x4               g114(.a(new_n189), .b(new_n193), .c(new_n194), .d(new_n190), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n182), .b(new_n210), .o1(new_n211));
  oai012aa1n02x5               g116(.a(new_n209), .b(new_n177), .c(new_n211), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  xnrc02aa1n02x5               g119(.a(\b[20] ), .b(\a[21] ), .out0(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  xnrc02aa1n02x5               g121(.a(\b[21] ), .b(\a[22] ), .out0(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n214), .c(new_n212), .d(new_n216), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n211), .clkout(new_n219));
  aoai13aa1n02x5               g124(.a(new_n216), .b(new_n208), .c(new_n199), .d(new_n219), .o1(new_n220));
  nona22aa1n02x4               g125(.a(new_n220), .b(new_n217), .c(new_n214), .out0(new_n221));
  nanp02aa1n02x5               g126(.a(new_n218), .b(new_n221), .o1(\s[22] ));
  norp02aa1n02x5               g127(.a(new_n217), .b(new_n215), .o1(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[22] ), .clkout(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\b[21] ), .clkout(new_n225));
  oao003aa1n02x5               g130(.a(new_n224), .b(new_n225), .c(new_n214), .carry(new_n226));
  aoi012aa1n02x5               g131(.a(new_n226), .b(new_n208), .c(new_n223), .o1(new_n227));
  nano22aa1n02x4               g132(.a(new_n183), .b(new_n223), .c(new_n210), .out0(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n228), .clkout(new_n229));
  oai012aa1n02x5               g134(.a(new_n227), .b(new_n177), .c(new_n229), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xnrc02aa1n02x5               g138(.a(\b[23] ), .b(\a[24] ), .out0(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n227), .clkout(new_n236));
  aoai13aa1n02x5               g141(.a(new_n233), .b(new_n236), .c(new_n199), .d(new_n228), .o1(new_n237));
  nona22aa1n02x4               g142(.a(new_n237), .b(new_n234), .c(new_n232), .out0(new_n238));
  nanp02aa1n02x5               g143(.a(new_n235), .b(new_n238), .o1(\s[24] ));
  norb02aa1n02x5               g144(.a(new_n233), .b(new_n234), .out0(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  nano32aa1n02x4               g146(.a(new_n241), .b(new_n182), .c(new_n223), .d(new_n210), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n242), .clkout(new_n243));
  aoai13aa1n02x5               g148(.a(new_n223), .b(new_n206), .c(new_n210), .d(new_n201), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n226), .clkout(new_n245));
  oai022aa1n02x5               g150(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n246));
  aob012aa1n02x5               g151(.a(new_n246), .b(\b[23] ), .c(\a[24] ), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n241), .c(new_n244), .d(new_n245), .o1(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n248), .clkout(new_n249));
  oai012aa1n02x5               g154(.a(new_n249), .b(new_n177), .c(new_n243), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  xnrc02aa1n02x5               g158(.a(\b[25] ), .b(\a[26] ), .out0(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n252), .c(new_n250), .d(new_n253), .o1(new_n255));
  aoai13aa1n02x5               g160(.a(new_n253), .b(new_n248), .c(new_n199), .d(new_n242), .o1(new_n256));
  nona22aa1n02x4               g161(.a(new_n256), .b(new_n254), .c(new_n252), .out0(new_n257));
  nanp02aa1n02x5               g162(.a(new_n255), .b(new_n257), .o1(\s[26] ));
  nanp02aa1n02x5               g163(.a(\b[25] ), .b(\a[26] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n253), .b(new_n254), .out0(new_n260));
  oai022aa1n02x5               g165(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n261));
  aoi022aa1n02x5               g166(.a(new_n248), .b(new_n260), .c(new_n259), .d(new_n261), .o1(new_n262));
  nano23aa1n02x4               g167(.a(new_n211), .b(new_n241), .c(new_n260), .d(new_n223), .out0(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n263), .clkout(new_n264));
  oai012aa1n02x5               g169(.a(new_n262), .b(new_n177), .c(new_n264), .o1(new_n265));
  xorb03aa1n02x5               g170(.a(new_n265), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[27] ), .b(\a[28] ), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n269), .b(new_n267), .c(new_n265), .d(new_n268), .o1(new_n270));
  aoai13aa1n02x5               g175(.a(new_n240), .b(new_n226), .c(new_n208), .d(new_n223), .o1(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n260), .clkout(new_n272));
  nanp02aa1n02x5               g177(.a(new_n261), .b(new_n259), .o1(new_n273));
  aoai13aa1n02x5               g178(.a(new_n273), .b(new_n272), .c(new_n271), .d(new_n247), .o1(new_n274));
  aoai13aa1n02x5               g179(.a(new_n268), .b(new_n274), .c(new_n199), .d(new_n263), .o1(new_n275));
  nona22aa1n02x4               g180(.a(new_n275), .b(new_n269), .c(new_n267), .out0(new_n276));
  nanp02aa1n02x5               g181(.a(new_n270), .b(new_n276), .o1(\s[28] ));
  norb02aa1n02x5               g182(.a(new_n268), .b(new_n269), .out0(new_n278));
  aoai13aa1n02x5               g183(.a(new_n278), .b(new_n274), .c(new_n199), .d(new_n263), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n267), .clkout(new_n280));
  oaoi03aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .c(new_n280), .o1(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  nona22aa1n02x4               g187(.a(new_n279), .b(new_n281), .c(new_n282), .out0(new_n283));
  aoai13aa1n02x5               g188(.a(new_n282), .b(new_n281), .c(new_n265), .d(new_n278), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(new_n284), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g190(.a(new_n118), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g191(.a(new_n268), .b(new_n282), .c(new_n269), .out0(new_n287));
  oao003aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .c(new_n280), .carry(new_n288));
  oaoi03aa1n02x5               g193(.a(\a[29] ), .b(\b[28] ), .c(new_n288), .o1(new_n289));
  xorc02aa1n02x5               g194(.a(\a[30] ), .b(\b[29] ), .out0(new_n290));
  160nm_ficinv00aa1n08x5       g195(.clk(new_n290), .clkout(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n289), .c(new_n265), .d(new_n287), .o1(new_n292));
  aoai13aa1n02x5               g197(.a(new_n287), .b(new_n274), .c(new_n199), .d(new_n263), .o1(new_n293));
  nona22aa1n02x4               g198(.a(new_n293), .b(new_n289), .c(new_n291), .out0(new_n294));
  nanp02aa1n02x5               g199(.a(new_n292), .b(new_n294), .o1(\s[30] ));
  nanp02aa1n02x5               g200(.a(new_n289), .b(new_n290), .o1(new_n296));
  oai012aa1n02x5               g201(.a(new_n296), .b(\b[29] ), .c(\a[30] ), .o1(new_n297));
  nano23aa1n02x4               g202(.a(new_n282), .b(new_n269), .c(new_n290), .d(new_n268), .out0(new_n298));
  aoai13aa1n02x5               g203(.a(new_n298), .b(new_n274), .c(new_n199), .d(new_n263), .o1(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  nona22aa1n02x4               g205(.a(new_n299), .b(new_n300), .c(new_n297), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n300), .b(new_n297), .c(new_n265), .d(new_n298), .o1(new_n302));
  nanp02aa1n02x5               g207(.a(new_n302), .b(new_n301), .o1(\s[31] ));
  xobna2aa1n03x5               g208(.a(new_n120), .b(new_n115), .c(new_n112), .out0(\s[3] ));
  oai012aa1n02x5               g209(.a(new_n115), .b(new_n120), .c(new_n111), .o1(new_n305));
  xnrb03aa1n02x5               g210(.a(new_n305), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  nanp02aa1n02x5               g211(.a(new_n121), .b(new_n113), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g213(.a(new_n107), .b(new_n307), .c(new_n122), .o1(new_n309));
  xnrb03aa1n02x5               g214(.a(new_n309), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_fiao0012aa1n02p5x5     g215(.a(new_n108), .b(new_n307), .c(new_n123), .o(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g217(.a(new_n102), .b(new_n311), .c(new_n103), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g219(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule



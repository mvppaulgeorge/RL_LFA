// Benchmark "adder" written by ABC on Thu Jul 11 13:03:58 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n160, new_n161, new_n162, new_n164, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n208, new_n209, new_n210, new_n211, new_n212, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n223,
    new_n224, new_n225, new_n226, new_n227, new_n229, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n243, new_n244, new_n245, new_n246,
    new_n247, new_n249, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n301, new_n304, new_n305,
    new_n307, new_n308, new_n310;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xnrc02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  aoi112aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n118));
  oab012aa1n02x4               g023(.a(new_n118), .b(\a[6] ), .c(\b[5] ), .out0(new_n119));
  oa0012aa1n02x5               g024(.a(new_n111), .b(new_n112), .c(new_n110), .o(new_n120));
  oabi12aa1n02x5               g025(.a(new_n120), .b(new_n114), .c(new_n119), .out0(new_n121));
  aoi112aa1n02x5               g026(.a(new_n121), .b(new_n98), .c(new_n109), .d(new_n117), .o1(new_n122));
  aoi112aa1n02x5               g027(.a(new_n122), .b(new_n97), .c(\a[9] ), .d(\b[8] ), .o1(new_n123));
  aoai13aa1n02x5               g028(.a(new_n97), .b(new_n122), .c(\b[8] ), .d(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n123), .out0(\s[10] ));
  norp02aa1n02x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  orn002aa1n02x5               g033(.a(\a[9] ), .b(\b[8] ), .o(new_n129));
  oaoi03aa1n02x5               g034(.a(\a[10] ), .b(\b[9] ), .c(new_n129), .o1(new_n130));
  oai012aa1n02x5               g035(.a(new_n128), .b(new_n123), .c(new_n130), .o1(new_n131));
  norp03aa1n02x5               g036(.a(new_n123), .b(new_n128), .c(new_n130), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n126), .clkout(new_n134));
  norp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n131), .c(new_n134), .out0(\s[12] ));
  nano23aa1n02x4               g043(.a(new_n126), .b(new_n135), .c(new_n136), .d(new_n127), .out0(new_n139));
  xnrc02aa1n02x5               g044(.a(\b[8] ), .b(\a[9] ), .out0(new_n140));
  nona22aa1n02x4               g045(.a(new_n139), .b(new_n97), .c(new_n140), .out0(new_n141));
  160nm_ficinv00aa1n08x5       g046(.clk(new_n141), .clkout(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n143));
  oaoi03aa1n02x5               g048(.a(\a[12] ), .b(\b[11] ), .c(new_n134), .o1(new_n144));
  aoi012aa1n02x5               g049(.a(new_n144), .b(new_n139), .c(new_n130), .o1(new_n145));
  norp02aa1n02x5               g050(.a(\b[12] ), .b(\a[13] ), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  nanb02aa1n02x5               g052(.a(new_n146), .b(new_n147), .out0(new_n148));
  xobna2aa1n03x5               g053(.a(new_n148), .b(new_n143), .c(new_n145), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n146), .clkout(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n148), .c(new_n143), .d(new_n145), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nano23aa1n02x4               g059(.a(new_n146), .b(new_n153), .c(new_n154), .d(new_n147), .out0(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  oai012aa1n02x5               g061(.a(new_n154), .b(new_n153), .c(new_n146), .o1(new_n157));
  aoai13aa1n02x5               g062(.a(new_n157), .b(new_n156), .c(new_n143), .d(new_n145), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  aoi012aa1n02x5               g066(.a(new_n160), .b(new_n158), .c(new_n161), .o1(new_n162));
  xnrb03aa1n02x5               g067(.a(new_n162), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  norp02aa1n02x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nano23aa1n02x4               g070(.a(new_n160), .b(new_n164), .c(new_n165), .d(new_n161), .out0(new_n166));
  nano22aa1n02x4               g071(.a(new_n141), .b(new_n155), .c(new_n166), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n168));
  norp02aa1n02x5               g073(.a(\b[9] ), .b(\a[10] ), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n170));
  oai112aa1n02x5               g075(.a(new_n137), .b(new_n128), .c(new_n170), .d(new_n169), .o1(new_n171));
  160nm_ficinv00aa1n08x5       g076(.clk(new_n144), .clkout(new_n172));
  aoai13aa1n02x5               g077(.a(new_n157), .b(new_n156), .c(new_n171), .d(new_n172), .o1(new_n173));
  oai012aa1n02x5               g078(.a(new_n165), .b(new_n164), .c(new_n160), .o1(new_n174));
  aobi12aa1n02x5               g079(.a(new_n174), .b(new_n173), .c(new_n166), .out0(new_n175));
  xnrc02aa1n02x5               g080(.a(\b[16] ), .b(\a[17] ), .out0(new_n176));
  xobna2aa1n03x5               g081(.a(new_n176), .b(new_n175), .c(new_n168), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g082(.clk(\a[17] ), .clkout(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(\b[16] ), .clkout(new_n179));
  nanp02aa1n02x5               g084(.a(new_n179), .b(new_n178), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n176), .c(new_n175), .d(new_n168), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g087(.clk(\a[18] ), .clkout(new_n183));
  xroi22aa1d04x5               g088(.a(new_n178), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(new_n184), .clkout(new_n185));
  norp02aa1n02x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  aoi013aa1n02x4               g092(.a(new_n186), .b(new_n187), .c(new_n178), .d(new_n179), .o1(new_n188));
  aoai13aa1n02x5               g093(.a(new_n188), .b(new_n185), .c(new_n175), .d(new_n168), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g095(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norp02aa1n02x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  aoi112aa1n02x5               g101(.a(new_n192), .b(new_n196), .c(new_n189), .d(new_n193), .o1(new_n197));
  aoai13aa1n02x5               g102(.a(new_n196), .b(new_n192), .c(new_n189), .d(new_n193), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(\s[20] ));
  nano23aa1n02x4               g104(.a(new_n192), .b(new_n194), .c(new_n195), .d(new_n193), .out0(new_n200));
  nona23aa1n02x4               g105(.a(new_n200), .b(new_n187), .c(new_n176), .d(new_n186), .out0(new_n201));
  nona23aa1n02x4               g106(.a(new_n195), .b(new_n193), .c(new_n192), .d(new_n194), .out0(new_n202));
  oai012aa1n02x5               g107(.a(new_n195), .b(new_n194), .c(new_n192), .o1(new_n203));
  oai012aa1n02x5               g108(.a(new_n203), .b(new_n202), .c(new_n188), .o1(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n204), .clkout(new_n205));
  aoai13aa1n02x5               g110(.a(new_n205), .b(new_n201), .c(new_n175), .d(new_n168), .o1(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g112(.a(\b[20] ), .b(\a[21] ), .o1(new_n208));
  xorc02aa1n02x5               g113(.a(\a[21] ), .b(\b[20] ), .out0(new_n209));
  xorc02aa1n02x5               g114(.a(\a[22] ), .b(\b[21] ), .out0(new_n210));
  aoi112aa1n02x5               g115(.a(new_n208), .b(new_n210), .c(new_n206), .d(new_n209), .o1(new_n211));
  aoai13aa1n02x5               g116(.a(new_n210), .b(new_n208), .c(new_n206), .d(new_n209), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n212), .b(new_n211), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g118(.clk(\a[21] ), .clkout(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(\a[22] ), .clkout(new_n215));
  xroi22aa1d04x5               g120(.a(new_n214), .b(\b[20] ), .c(new_n215), .d(\b[21] ), .out0(new_n216));
  nanp03aa1n02x5               g121(.a(new_n216), .b(new_n184), .c(new_n200), .o1(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(\b[21] ), .clkout(new_n218));
  oao003aa1n02x5               g123(.a(new_n215), .b(new_n218), .c(new_n208), .carry(new_n219));
  aoi012aa1n02x5               g124(.a(new_n219), .b(new_n204), .c(new_n216), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n217), .c(new_n175), .d(new_n168), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g127(.a(\b[22] ), .b(\a[23] ), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[23] ), .b(\b[22] ), .out0(new_n224));
  xorc02aa1n02x5               g129(.a(\a[24] ), .b(\b[23] ), .out0(new_n225));
  aoi112aa1n02x5               g130(.a(new_n223), .b(new_n225), .c(new_n221), .d(new_n224), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n225), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(\s[24] ));
  and002aa1n02x5               g133(.a(new_n225), .b(new_n224), .o(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(new_n229), .clkout(new_n230));
  nano32aa1n02x4               g135(.a(new_n230), .b(new_n216), .c(new_n184), .d(new_n200), .out0(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n231), .clkout(new_n232));
  oaoi03aa1n02x5               g137(.a(\a[18] ), .b(\b[17] ), .c(new_n180), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n203), .clkout(new_n234));
  aoai13aa1n02x5               g139(.a(new_n216), .b(new_n234), .c(new_n200), .d(new_n233), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n219), .clkout(new_n236));
  oai022aa1n02x5               g141(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n237));
  aob012aa1n02x5               g142(.a(new_n237), .b(\b[23] ), .c(\a[24] ), .out0(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n230), .c(new_n235), .d(new_n236), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n232), .c(new_n175), .d(new_n168), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g147(.a(\b[24] ), .b(\a[25] ), .o1(new_n243));
  xorc02aa1n02x5               g148(.a(\a[25] ), .b(\b[24] ), .out0(new_n244));
  xorc02aa1n02x5               g149(.a(\a[26] ), .b(\b[25] ), .out0(new_n245));
  aoi112aa1n02x5               g150(.a(new_n243), .b(new_n245), .c(new_n241), .d(new_n244), .o1(new_n246));
  aoai13aa1n02x5               g151(.a(new_n245), .b(new_n243), .c(new_n241), .d(new_n244), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n246), .out0(\s[26] ));
  nanp02aa1n02x5               g153(.a(new_n109), .b(new_n117), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n121), .clkout(new_n250));
  nanp02aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n166), .clkout(new_n252));
  aoai13aa1n02x5               g157(.a(new_n155), .b(new_n144), .c(new_n139), .d(new_n130), .o1(new_n253));
  aoai13aa1n02x5               g158(.a(new_n174), .b(new_n252), .c(new_n253), .d(new_n157), .o1(new_n254));
  and002aa1n02x5               g159(.a(new_n245), .b(new_n244), .o(new_n255));
  nano22aa1n02x4               g160(.a(new_n217), .b(new_n229), .c(new_n255), .out0(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n254), .c(new_n251), .d(new_n167), .o1(new_n257));
  oai022aa1n02x5               g162(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n258));
  aob012aa1n02x5               g163(.a(new_n258), .b(\b[25] ), .c(\a[26] ), .out0(new_n259));
  aobi12aa1n02x5               g164(.a(new_n259), .b(new_n239), .c(new_n255), .out0(new_n260));
  xorc02aa1n02x5               g165(.a(\a[27] ), .b(\b[26] ), .out0(new_n261));
  xnbna2aa1n03x5               g166(.a(new_n261), .b(new_n257), .c(new_n260), .out0(\s[27] ));
  norp02aa1n02x5               g167(.a(\b[26] ), .b(\a[27] ), .o1(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n263), .clkout(new_n264));
  aobi12aa1n02x5               g169(.a(new_n261), .b(new_n257), .c(new_n260), .out0(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[27] ), .b(\a[28] ), .out0(new_n266));
  nano22aa1n02x4               g171(.a(new_n265), .b(new_n264), .c(new_n266), .out0(new_n267));
  nanp02aa1n02x5               g172(.a(new_n175), .b(new_n168), .o1(new_n268));
  aoai13aa1n02x5               g173(.a(new_n229), .b(new_n219), .c(new_n204), .d(new_n216), .o1(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n255), .clkout(new_n270));
  aoai13aa1n02x5               g175(.a(new_n259), .b(new_n270), .c(new_n269), .d(new_n238), .o1(new_n271));
  aoai13aa1n02x5               g176(.a(new_n261), .b(new_n271), .c(new_n268), .d(new_n256), .o1(new_n272));
  aoi012aa1n02x5               g177(.a(new_n266), .b(new_n272), .c(new_n264), .o1(new_n273));
  norp02aa1n02x5               g178(.a(new_n273), .b(new_n267), .o1(\s[28] ));
  norb02aa1n02x5               g179(.a(new_n261), .b(new_n266), .out0(new_n275));
  aoai13aa1n02x5               g180(.a(new_n275), .b(new_n271), .c(new_n268), .d(new_n256), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[28] ), .b(\b[27] ), .c(new_n264), .carry(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[28] ), .b(\a[29] ), .out0(new_n278));
  aoi012aa1n02x5               g183(.a(new_n278), .b(new_n276), .c(new_n277), .o1(new_n279));
  aobi12aa1n02x5               g184(.a(new_n275), .b(new_n257), .c(new_n260), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n280), .b(new_n277), .c(new_n278), .out0(new_n281));
  norp02aa1n02x5               g186(.a(new_n279), .b(new_n281), .o1(\s[29] ));
  xorb03aa1n02x5               g187(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g188(.a(new_n261), .b(new_n278), .c(new_n266), .out0(new_n284));
  aoai13aa1n02x5               g189(.a(new_n284), .b(new_n271), .c(new_n268), .d(new_n256), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[29] ), .b(\b[28] ), .c(new_n277), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[29] ), .b(\a[30] ), .out0(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(new_n285), .c(new_n286), .o1(new_n288));
  aobi12aa1n02x5               g193(.a(new_n284), .b(new_n257), .c(new_n260), .out0(new_n289));
  nano22aa1n02x4               g194(.a(new_n289), .b(new_n286), .c(new_n287), .out0(new_n290));
  norp02aa1n02x5               g195(.a(new_n288), .b(new_n290), .o1(\s[30] ));
  norb02aa1n02x5               g196(.a(new_n284), .b(new_n287), .out0(new_n292));
  aobi12aa1n02x5               g197(.a(new_n292), .b(new_n257), .c(new_n260), .out0(new_n293));
  oao003aa1n02x5               g198(.a(\a[30] ), .b(\b[29] ), .c(new_n286), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[30] ), .b(\a[31] ), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n293), .b(new_n294), .c(new_n295), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n292), .b(new_n271), .c(new_n268), .d(new_n256), .o1(new_n297));
  aoi012aa1n02x5               g202(.a(new_n295), .b(new_n297), .c(new_n294), .o1(new_n298));
  norp02aa1n02x5               g203(.a(new_n298), .b(new_n296), .o1(\s[31] ));
  xnrb03aa1n02x5               g204(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g205(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n301));
  xorb03aa1n02x5               g206(.a(new_n301), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g207(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g208(.a(new_n116), .b(new_n109), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n304), .b(\b[4] ), .c(\a[5] ), .o1(new_n305));
  xorb03aa1n02x5               g210(.a(new_n305), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  orn002aa1n02x5               g211(.a(\a[5] ), .b(\b[4] ), .o(new_n307));
  aoai13aa1n02x5               g212(.a(new_n119), .b(new_n115), .c(new_n304), .d(new_n307), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g214(.a(new_n112), .b(new_n308), .c(new_n113), .o1(new_n310));
  xnrb03aa1n02x5               g215(.a(new_n310), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g216(.a(new_n140), .b(new_n249), .c(new_n250), .out0(\s[9] ));
endmodule


